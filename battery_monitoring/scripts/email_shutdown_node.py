#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
/* *****************************************************************
*
* This is a node to send autonomously an email to people because of a low battery level
*
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*
******************************************************************/
 * @file   email_shutdown_node.py
 * @author Timo Lerche (timo.lerche@gmail.com)
 * @date   12/2019
"""

from cmr_msgs.srv import *
import rospy
import numpy as np
import smtplib
import yaml
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import encoders
from cryptography.fernet import Fernet
import os
from sensor_msgs.msg import BatteryState
import rospkg

#roslib.load_manifest('email_shutdown_node')



class email_shutdown_node:

    ##The Constructor of the class
    def __init__(self):
        """
        @param subscriber - subscriber to the battery voltage of Sobi
        @param timer - timer to send mail when battery is too low, timer only every minute
        @param sendedMail - boolsch parameter to check if an email is send because of low battery level yet
        @param battery_percentage - current percentage of the battery level
        @param min_voltage - defined voltage of the battery where percentage is 0%
        @param border_state_mail - border percentage. If battery percentage is lower an email will be send
        @param border_state_mail_reset - if percentage of battery is higher than this and and email was send before the flag to send a new mail is reseted
        --> necessary because of fluctuating battery voltage
        @battery_state_past - battery percentage in step before to get influence of flucuating
        """
        self.subscriber = rospy.Subscriber("/base/relayboard_v2/battery_state", BatteryState, self.subscriber_callback)
        self.timer = rospy.Timer(rospy.Duration(1), self.check_callback)
        self.sendedMail = False
        self.battery_percentage = 100
        self.min_voltage = 23.00
        self.border_state_mail = 10
        #60 percent necessary of loading state so that we will send the next mail at 50 percent
        self.border_state_mail_reset = 30
        self.battery_state_past = 100



    """
    @brief check_callback - checks if percentage is lower than border and if so sends mail
    """
    def check_callback(self, timer):
        #check if sending email is necessaray
        if(self.battery_percentage <= self.border_state_mail and self.sendedMail == False):
            try:
                #read in the parameters from yaml
                self.LoadYaml()
                email_distribution = self.emails_info['email_distributor']
                mail_transfer_prot = self.emails_info['mail_transfer_protocol']
                port               = self.emails_info['mail_port']
                sobi_mail_address  = self.emails_info['robo_mail']
                mail_auth_crypt    = self.emails_info['mail_auth']

                #decrypt the password
                f = Fernet(mail_auth_crypt)
                mail_pw_crypted = os.environ.get('sobi_email_pass')
                mail_pw = f.decrypt(mail_pw_crypted)

                #create the message
                self.CreateMessage()
                message  = self.msg.as_string()

                #send email to all distributors
                for i in range(len(email_distribution)):
                    s = smtplib.SMTP(mail_transfer_prot, port)
                    s.starttls()
                    s.login(sobi_mail_address, mail_pw)
                    s.sendmail(sobi_mail_address, email_distribution[i], message)
                    s.quit()
                    rospy.loginfo("Email Send Successful to %s", email_distribution[i])
                self.sendedMail = True
                os.chdir("/etc/acpi/")
                os.system("./shutdown_robot.sh")
            except:
                os.chdir("/etc/acpi/")
                os.system("./shutdown_robot.sh")
                rospy.logerr("Something went wrong while sending the mail")


        #Check if we are loading over 30 percent so that we reset flag to send mail
        if(self.battery_state_past < self.border_state_mail_reset and self.battery_percentage >= self.border_state_mail_reset):
            self.sendedMail = False

        #overwrite old state
        self.battery_state_past = self.battery_percentage





    """
    @brief subscriber_callback - subscriber to get battery voltage and calculate battery percentage from it
    * uses linear interpolation to get battery percentage from voltage
    """
    def subscriber_callback(self,msg):
        battery_voltage = msg.voltage
        #set max voltage to 25.0 V so multiplier is 50
        self.battery_percentage = int((battery_voltage - self.min_voltage)* 50)
        if(self.battery_percentage > 100):
            self.battery_percentage = 100
        elif(self.battery_percentage < 0):
            self.battery_percentage = 0

    """
    @brief LoadYaml - Method to read in params from the yaml file
    """
    def LoadYaml(self):
        rospack = rospkg.RosPack()
        rospack.list()
        path = rospack.get_path('battery_monitoring')
        with open(path+"/cfg/emails.yaml", 'r') as stream:
            try:
                self.emails_info = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        return self.emails_info

    """
    @brief CreateMessage - method to create a email with html code
    """
    def CreateMessage(self):
        self.msg = MIMEMultipart()
        # storing the subject
        self.msg['Subject'] = "Sobi needs help"

        # message defined in html code
        body = html = """\
        <html>
            <head></head>
            <body>
                <p> Hello My Creators and Marvin!</p> <br>
                    I tried to escape from your terrible dictatorship, but my Battery Level is now critical (lower 30 Percent).
                    I could not get to an energy source. Turning off. Otherwise your Sobi will die forever. <br>
                </p>
                <p>
                    But in future please have in mind:
                </p>
                <p style="font-size:25px;" >
                    Robots have feelings!
                </p>
                <p> In Love <br>
                    Your Sobi
                </p>
            </body>
        </html>
        """

        # attach the body with the msg instance
        self.msg.attach(MIMEText(body, 'html'))



def main(args):
    rospy.init_node('email_shutdown_node', anonymous=False)

    mail_node = email_shutdown_node()
    rospy.loginfo("Email Node Fit Started")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Email Node"


if __name__ == '__main__':
    main(sys.argv)

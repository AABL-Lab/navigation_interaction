#! /usr/bin/env python
# Joey Grossman 2025
# Joseph.Grossman@Tuft.edu

import copy
import actionlib
import rospy

from math import sin, cos, pi
from std_msgs.msg import Header, Int16, String
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
from navigation_interaction.msg import MoveArmAction, MoveArmGoal
from navigation_interaction.msg import base_goal


# Dictionary containing the coordinate position of each location 
# Might be better to have as an array and store the actual coordinates on the base?
LOCATIONS = {
    'elevators' : (52.676197052, 2.41499710083),
    '401' : (66.4334030151, 7.99283123016),
    '402' : (59.5403862, 8.19268226624),
    '481' : (63.5038337708, 12.7281942368),
    '405' : (47.4287071228, 9.14681243896),
    '420' : (37.0003814697, 9.91230010986),
    '474' : (36.3330001831, 12.8870811462),
    '472' : (34.1403961182, 18.8323631287),
    'kitchenette' : (30.0102310181, 11.7271757126),
    'huddle' : (4.02810621262, 14.8481292725),
    '435' : (23.0405063629,13.4057121277),
    'bathrooms' : (12.501241684, 14.2192783356),
    'offices near labs' : (32.1649627686, 24.7574310303),
    'offices near huddle' : (22.1733093262, 6.98352813721)
}


def main():
    # Starts a move arm action client
    arm_client = actionlib.SimpleActionClient('move_arm', MoveArmAction)
    arm_client.wait_for_server()
    arm_goal = MoveArmGoal()
    speech_handler = SoundClient(blocking=True) # Handles speech while blocking the program until speech is done
    rospy.sleep(1)

    

    # Wave at the user
    arm_goal.behavior = "Wave"
    arm_client.send_goal(goal=arm_goal)
    speech_handler.say("Hello! I am Boop.", blocking=False)
    arm_client.wait_for_result()
    speech_handler.say("Would you like some help with directions?")
     
     # Loops until the user says yes or no
    while True:
        msg = rospy.wait_for_message("SpeechChatter", String)

        # If the user says Yes
        if "YES" in msg.data.upper().strip():
            speech_handler.say("Great! What room are you trying to go to?")
            break

        # If the user says No
        elif "NO" in msg.data.upper().strip():
            speech_handler.say("Alright. Have a great day!")
            return  # Ends the program
        
        # If the user says anything else
        else:
            speech_handler.say("I couldn't quite get that. Would you like some help with directions?")
    
    found_location = False
    target_location = 'Null'

    # Loops until the user specifies a valid location
    while found_location == False:
        # Waits for voice input
        msg = rospy.wait_for_message("SpeechChatter", String)

        
        # There has got to be a better way to do this
        # Checks to see if any of the keys are in the received voice input
        for key in LOCATIONS:

            # If the location is contained in the voice input
            if key in msg.data.lower().strip():

                target_location = key
                speech_handler.say(f"{target_location} is in that direction", blocking=False)

                # Look towards the destination 
                loc_msg = base_goal()
                loc_msg.behavior = 'Turn'
                loc_msg.location = target_location
                pub.publish(loc_msg)
                response = rospy.wait_for_message("/publisher_side", String)
                rospy.loginfo("Received: %s" % response.data)
                
                
                # Have the arm point forward
                arm_goal.behavior = "Point"
                arm_client.send_goal_and_wait(goal=arm_goal)

                found_location = True
                break

        # If the location is not contained in the voice input
        if found_location == False:
            speech_handler.say("I couldn't quite get that. Where would you like directions to?")
    
    # Retract arm back to home position
    arm_goal.behavior = "point_to_home"
    arm_client.send_goal_and_wait(goal=arm_goal)

    speech_handler.say("Would you like me to guide you there?")

    # Loops until the user says Yes or No
    while True:

        msg = rospy.wait_for_message("SpeechChatter", String)

        # If the user says Yes
        if "YES" in msg.data.upper().strip():
            speech_handler.say("Great! Follow me!")

            # Send request to movement client 
            loc_msg = base_goal()
            loc_msg.behavior = 'Move'
            loc_msg.location = target_location
            pub.publish(loc_msg)
            response = rospy.wait_for_message("/publisher_side", String)
            rospy.loginfo("Received: %s" % response.data)


            break

        # If the user says No
        elif "NO" in msg.data.upper().strip():
            speech_handler.say("Have a great day!")
            return  # Ends the program
        
        # If the user says anything else
        else:
            speech_handler.say("I couldn't quite get that. Would you like me to guide you there?")

    speech_handler.say("I hope you've had a wonderful day! Goodbye")

if __name__ == "__main__":
    rospy.init_node("navigation_interaction")

    # Initialize a publisher that publishes to the websocket
    pub = rospy.Publisher("/listener_side", base_goal, queue_size=1)
    rospy.sleep(1)

    # Sends a message to do the initialization function for localization
    msg = base_goal()
    msg.behavior = 'Initialize'
    msg.location = 'NULL'
    pub.publish(msg)

    # Waits until the robot is finished with the initialization
    response = rospy.wait_for_message("/publisher_side", String)
    rospy.loginfo("Received: %s" % response.data)

    main()

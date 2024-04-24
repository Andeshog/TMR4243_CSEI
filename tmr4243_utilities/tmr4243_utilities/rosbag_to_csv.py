#!/usr/bin/env python3
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import csv
import os
import sys

def connect(sqlite_file):
    conn = sqlite3.connect(sqlite_file)
    c = conn.cursor()
    return conn, c

def close(conn):
    conn.close()

def getAllElements(cursor, table_name, print_out=False):
    """ Returns a dictionary with all elements of the table database.
    """
    # Get elements from table "table_name"
    cursor.execute('SELECT * from({})'.format(table_name))
    records = cursor.fetchall()
    if print_out:
        print("\nAll elements:")
        for row in records:
            print(row)
    return records

def isTopic(cursor, topic_name, print_out=False):
    """ Returns topic_name header if it exists. If it doesn't, returns empty.
        It returns the last topic found with this name.
    """
    boolIsTopic = False
    topicFound = []

    # Get all records for 'topics'
    records = getAllElements(cursor, 'topics', print_out=False)

    # Look for specific 'topic_name' in 'records'
    for row in records:
        if(row[1] == topic_name): # 1 is 'name' TODO
            boolIsTopic = True
            topicFound = row
    if print_out:
        if boolIsTopic:
             # 1 is 'name', 0 is 'id' TODO
            print('\nTopic named', topicFound[1], ' exists at id ', topicFound[0] ,'\n')
        else:
            print('\nTopic', topic_name ,'could not be found. \n')

    return topicFound

def getAllMessagesInTopic(cursor, topic_name, print_out=False):
    """ Returns all timestamps and messages at that topic.
    There is no deserialization for the BLOB data.
    """
    count = 0
    timestamps = []
    messages = []

    # Find if topic exists and its id
    topicFound = isTopic(cursor, topic_name, print_out=False)

    # If not find return empty
    if not topicFound:
        print('Topic', topic_name ,'could not be found. \n')
    else:
        records = getAllElements(cursor, 'messages', print_out=False)

        # Look for message with the same id from the topic
        for row in records:
            if row[1] == topicFound[0]:     # 1 and 0 is 'topic_id' TODO
                count = count + 1           # count messages for this topic
                timestamps.append(row[2])   # 2 is for timestamp TODO
                messages.append(row[3])     # 3 is for all messages

        # Print
        if print_out:
            print('\nThere are ', count, 'messages in ', topicFound[1])

    return timestamps, messages

def getAllTopicsNames(cursor, print_out=False):
    """ Returns all topics names.
    """
    topicNames = []
    # Get all records for 'topics'
    records = getAllElements(cursor, 'topics', print_out=False)

    # Save all topics names
    for row in records:
        topicNames.append(row[1])  # 1 is for topic name TODO
    if print_out:
        print('\nTopics names are:')
        print(topicNames)

    return topicNames

def getAllMsgsTypes(cursor, print_out=False):
    """ Returns all messages types.
    """
    msgsTypes = []
    # Get all records for 'topics'
    records = getAllElements(cursor, 'topics', print_out=False)

    # Save all message types
    for row in records:
        msgsTypes.append(row[2])  # 2 is for message type TODO
    if print_out:
        print('\nMessages types are:')
        print(msgsTypes)

    return msgsTypes

if __name__ == "__main__":
        csv_directory = "/home/andeshog/sim_ws/csv_files/Sim/Observer/" #sys.argv[1]
        bag_file = "/home/andeshog/sim_ws/bags/ObserverRecordings/Sim/DeadReckoningStraightLine/DRStraightLine.db3" #sys.argv[2]
        print(f"Converting {bag_file} to CSV files in {csv_directory}")

        csv_directory = csv_directory + os.path.basename(bag_file).split('.')[0]
        if not os.path.exists(csv_directory):
            os.makedirs(csv_directory)

        ### connect to the database
        conn, c = connect(bag_file)

        ### get all topics names and types
        topic_names = getAllTopicsNames(c, print_out=False)
        topic_types = getAllMsgsTypes(c, print_out=False)

        # Create a map for quicker lookup
        type_map = {topic_names[i]:topic_types[i] for i in range(len(topic_types))}
        
        for topic_name in topic_names:
            
            if topic_name == "/tf_static":
                continue
            elif topic_name == "/tf":
                continue
            elif topic_name == "/robot_description":
                continue
            elif topic_name == "/rosout":
                continue

            ### get all timestamps and all messages
            t, msgs = getAllMessagesInTopic(c, topic_name, print_out=True)
            if len(t) == 0:
                continue
            # # Deserialize the message
            msg_type = get_message(type_map[topic_name])  # Assuming type_map is a dictionary mapping topic names to message types

            # Create a CSV file for the topic
            csv_file = os.path.join(csv_directory, topic_name.replace('/', '_') + ".csv")
            print(f"Creating CSV file: {csv_file}")

            # Open the CSV file for writing
            with open(csv_file, 'w', newline='') as csvfile:
                # Create a CSV writer
                csv_writer = csv.writer(csvfile)

                if topic_name == "/CSEI/observer/state":
                    csv_writer.writerow(["Timestamp","eta.x", "eta.y", "eta.z", "nu.u", "nu.v", "nu.w", "bias.x", "bias.y", "bias.z"])
                    for timestamp, message in zip(t, msgs):
                            deserialized_msg = deserialize_message(message, msg_type)
                            # seconds = timestamp // 10**9
                            # nanoseconds = timestamp % 10**9
                            # timestamp = f"{seconds}.{nanoseconds}"
                            timestamp = timestamp / 10**9
                            csv_writer.writerow([timestamp, deserialized_msg.eta[0], deserialized_msg.eta[1], deserialized_msg.eta[2], deserialized_msg.nu[0], deserialized_msg.nu[1], deserialized_msg.nu[2], deserialized_msg.bias[0], deserialized_msg.bias[1], deserialized_msg.bias[2]])
                elif topic_name == "/CSEI/control/u_cmd":
                    csv_writer.writerow(["Timestamp","u_cmd.1", "u_cmd.2", "u_cmd.3", "u_cmd.4","u_cmd.5"])
                    for timestamp, message in zip(t, msgs):
                            deserialized_msg = deserialize_message(message, msg_type)
                            timestamp = timestamp / 10**9
                            csv_writer.writerow([timestamp, deserialized_msg.data[0], deserialized_msg.data[1], deserialized_msg.data[2], deserialized_msg.data[3], deserialized_msg.data[4]])
                elif topic_name == "/CSEI/state/tau":
                    csv_writer.writerow(["Timestamp","tau.1", "tau.2", "tau.3"])
                    for timestamp, message in zip(t, msgs):
                            deserialized_msg = deserialize_message(message, msg_type)
                            timestamp = timestamp / 10**9
                            csv_writer.writerow([timestamp, deserialized_msg.data[0], deserialized_msg.data[1], deserialized_msg.data[2]])
                elif topic_name == "/CSEI/state/eta":
                    csv_writer.writerow(["Timestamp","eta.1", "eta.2", "eta.3"])
                    for timestamp, message in zip(t, msgs):
                            deserialized_msg = deserialize_message(message, msg_type)
                            timestamp = timestamp / 10**9
                            csv_writer.writerow([timestamp, deserialized_msg.data[0], deserialized_msg.data[1], deserialized_msg.data[2]])
                elif topic_name == "/CSEI/state/psi":
                    csv_writer.writerow(["Timestamp","psi"])
                    for timestamp, message in zip(t, msgs):
                            deserialized_msg = deserialize_message(message, msg_type)
                            timestamp = timestamp / 10**9
                            csv_writer.writerow([timestamp, deserialized_msg.data])
                elif topic_name == "/CSEI/thrusters/tunnel/command":
                    csv_writer.writerow(["Timestamp","force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"])
                    for timestamp, message in zip(t, msgs):
                            deserialized_msg = deserialize_message(message, msg_type)
                            timestamp = timestamp / 10**9
                            csv_writer.writerow([timestamp, deserialized_msg.force.x, deserialized_msg.force.y, deserialized_msg.force.z, deserialized_msg.torque.x, deserialized_msg.torque.y, deserialized_msg.torque.z])
                elif topic_name == "/CSEI/thrusters/starboard/command":
                    csv_writer.writerow(["Timestamp","force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"])
                    for timestamp, message in zip(t, msgs):
                            deserialized_msg = deserialize_message(message, msg_type)
                            timestamp = timestamp / 10**9
                            csv_writer.writerow([timestamp, deserialized_msg.force.x, deserialized_msg.force.y, deserialized_msg.force.z, deserialized_msg.torque.x, deserialized_msg.torque.y, deserialized_msg.torque.z])
                elif topic_name == "/CSEI/thrusters/port/command":
                    csv_writer.writerow(["Timestamp","force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"])
                    for timestamp, message in zip(t, msgs):
                            deserialized_msg = deserialize_message(message, msg_type)
                            timestamp = timestamp / 10**9
                            csv_writer.writerow([timestamp, deserialized_msg.force.x, deserialized_msg.force.y, deserialized_msg.force.z, deserialized_msg.torque.x, deserialized_msg.torque.y, deserialized_msg.torque.z])
                elif topic_name == "/CSEI/control/DPref":
                    csv_writer.writerow(["Timestamp", "eta_d.x", "eta_d.y", "eta_d.z", "eta_ds.x", "eta_ds.y", "eta_ds.z", "w", "v_s", "v_ss"])
                    for timestamp, message in zip(t, msgs):
                        deserialized_msg = deserialize_message(message, msg_type)
                        dm = deserialized_msg
                        timestamp = timestamp / 10**9  # Convert timestamp to seconds
                        csv_writer.writerow([timestamp, dm.eta_d[0], dm.eta_d[1], dm.eta_d[2], dm.eta_ds[0], dm.eta_ds[1], dm.eta_ds[2], dm.w, dm.v_s, dm.v_ss])
                elif topic_name == "/CSEI/control/reference":
                    csv_writer.writerow(["Timestamp", "eta_d.x", "eta_d.y", "eta_d.z", "eta_ds.x", "eta_ds.y", "eta_ds.z", "eta_ds2.x", "eta_ds2.y", "eta_ds2.z", "w", "v_s", "v_ss"])
                    for timestamp, message in zip(t, msgs):
                        deserialized_msg = deserialize_message(message, msg_type)
                        dm = deserialized_msg
                        timestamp = timestamp / 10**9
                        csv_writer.writerow([timestamp, dm.eta_d[0], dm.eta_d[1], dm.eta_d[2], dm.eta_ds[0], dm.eta_ds[1], dm.eta_ds[2], dm.eta_ds2[0], dm.eta_ds2[1], dm.eta_ds2[2], dm.w, dm.v_s, dm.v_ss])
                else:
                    csv_writer.writerow(["Timestamp","data"])
                    for timestamp, message in zip(t, msgs):
                            deserialized_msg = deserialize_message(message, msg_type)
                            timestamp = timestamp / 10**9
                            csv_writer.writerow([timestamp, deserialized_msg])

        ### close connection to the database
        close(conn)
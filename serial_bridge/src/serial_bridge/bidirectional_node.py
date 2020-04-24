#!/usr/bin/env python3.5
import roslib;
import rospy

import threading
from serial import *
import StringIO
from collections import OrderedDict
from rosserial_msgs.msg import *
from rosserial_msgs.srv import *
from serial_bridge.msg import Time

import diagnostic_msgs.msg

import socket
import time
import struct
import yaml

import zlib

__author__ = "juancamilog@gmail.com (Juan Camilo Gamboa Higuera)"
__author__ = "maxsvetlik@utexas.edu (Max Svetlik)"

def load_pkg_module(package, directory):
    #check if its in the python path
    in_path = False
    path = sys.path
    pkg_src = package+'/src' #check for the source directory which
                             # is added to path by roslib boostrapping
    for entry in sys.path:
        if pkg_src in entry:
            in_path = True
    if not in_path:
        roslib.load_manifest(package)
    try:
        m = __import__( package + '.' + directory )
    except:
        rospy.logerr( "Cannot import package : %s"% package )
        rospy.logerr( "sys.path was " + str(path) )
        return None
    return m

def load_message(package, message):
    m = load_pkg_module(package, 'msg')
    m2 = getattr(m, 'msg')
    return getattr(m2, message)

class Publisher:
    """
        Publisher forwards messages from the remote device to local ROS.
    """
    def __init__(self, topic_info):
        """ Create a new publisher. """
        self.topic = topic_info.topic_name
        self.message_type = topic_info.message_type

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = load_message(package, message)
        if self.message._md5sum == topic_info.md5sum:
            self.publisher = rospy.Publisher(self.topic, self.message, queue_size=1)
        else:
            raise Exception('Checksum does not match: ' + self.message._md5sum + ',' + topic_info.md5sum)

    def handlePacket(self, data):
        """ Forward message to ROS network. """
        m = self.message()
        m.deserialize(data)
        self.publisher.publish(m)


class Subscriber:
    """
        Subscriber forwards messages from local ROS to the remote device.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.id = topic_info.topic_id
        self.message_type = topic_info.message_type
        self.parent = parent

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = load_message(package, message)
        if self.message._md5sum == topic_info.md5sum:
            self.sub = rospy.Subscriber(self.topic, self.message, self.callback, queue_size=1)
        else:
            raise Exception('Checksum does not match: ' + self.message._md5sum + ',' + topic_info.md5sum)

    def unregister(self):
        rospy.loginfo("Removing subscriber: %s", self.topic)
        self.sub.unregister()

    def callback(self, msg):
        """ Forward message to serial device. """
        data_buffer = StringIO.StringIO()
        msg.serialize(data_buffer)
        # only send data if the other side is alive
        if self.parent.synced is not False:
            self.parent.send(self.id, data_buffer.getvalue())


class ServiceServer:
    """
        ServiceServer responds to requests from ROS.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.message_type = topic_info.message_type
        self.parent = parent

        # find message type
        package, service = topic_info.message_type.split('/')
        s = load_pkg_module(package, 'srv')
        s = getattr(s, 'srv')
        self.mreq = getattr(s, service+"Request")
        self.mres = getattr(s, service+"Response")
        srv = getattr(s, service)
        self.service = rospy.Service(self.topic, srv, self.callback)

        # response message
        self.data = None

    def unregister(self):
        rospy.loginfo("Removing service: %s", self.topic)
        self.service.shutdown()

    def callback(self, req):
        """ Forward request to serial device. """
        data_buffer = StringIO.StringIO()
        req.serialize(data_buffer)
        self.response = None
        if self.parent.send(self.id, data_buffer.getvalue()) >= 0:
            while self.response == None:
                pass
        return self.response

    def handlePacket(self, data):
        """ Forward response to ROS network. """
        r = self.mres()
        r.deserialize(data)
        self.response = r


class ServiceClient:
    """
        ServiceServer responds to requests from ROS.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.message_type = topic_info.message_type
        self.parent = parent

        # find message type
        package, service = topic_info.message_type.split('/')
        s = load_pkg_module(package, 'srv')
        s = getattr(s, 'srv')
        self.mreq = getattr(s, service+"Request")
        self.mres = getattr(s, service+"Response")
        srv = getattr(s, service)
        rospy.loginfo("Starting service client, waiting for service '" + self.topic + "'")
        rospy.wait_for_service(self.topic)
        self.proxy = rospy.ServiceProxy(self.topic, srv)

    def handlePacket(self, data):
        """ Forward request to ROS network. """
        req = self.mreq()
        req.deserialize(data)
        # call service proxy
        resp = self.proxy(req)
        # serialize and publish
        data_buffer = StringIO.StringIO()
        resp.serialize(data_buffer)
        self.parent.send(self.id, data_buffer.getvalue())

class BidirectionalNode:
    """
        ServiceServer responds to requests from the serial device.
    """

    def __init__(self, port=None, baud=1000000, timeout=5.0, compressed=False):
        """ Initialize node, connect to bus, attempt to negotiate topics. """
        self.mutex = threading.Lock()
        self.lastsync = rospy.Time(0)
        self.lastsync_lost = rospy.Time(0)
        self.timeout = timeout
        self.synced = False

        self.compressed = compressed

        self.pub_diagnostics = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray, queue_size=1)

        if port== None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'recv') or hasattr(port, 'read'):
            #assume its a filelike object
            self.port=port
        else:
            # open a specific port
            try:
                self.port = Serial(port, baud, timeout=self.timeout*0.5)
            except SerialException as e:
                rospy.logerr("Error opening serial: %s", e)
                rospy.signal_shutdown("Error opening serial: %s" % e)
                raise SystemExit

        #self.port.timeout = 0.01  # Edit the port timeout

        # hydro introduces protocol ver2 which must match node_handle.h
        # The protocol version is sent as the 2nd sync byte emitted by each end
        self.protocol_ver1 = '\xff'
        self.protocol_ver2 = '\xfe'
        self.protocol_ver = self.protocol_ver2

        self.publishers = OrderedDict()  # id:Publishers
        self.subscribers = dict() # topic:Subscriber
        self.services = dict()    # topic:Service
        self.remote_time = dict() # hostname:time_to_last_timesync
        self.remote_time_offset = dict() # hostname:time_offset i.e. localtime - remote hostname's time

        # TODO determine if these need a definite value and if so, what it is
        self.buffer_out = 1024
        self.buffer_in = 1024

        self.callbacks = dict()
        # endpoints for creating new pubs/subs
        self.callbacks[TopicInfo.ID_PUBLISHER] = self.setupPublisher
        self.callbacks[TopicInfo.ID_SUBSCRIBER] = self.setupSubscriber
        # service client/servers have 2 creation endpoints (a publisher and a subscriber)
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_PUBLISHER] = self.setupServiceServerPublisher
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_SUBSCRIBER] = self.setupServiceServerSubscriber
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_PUBLISHER] = self.setupServiceClientPublisher
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_SUBSCRIBER] = self.setupServiceClientSubscriber
        # custom endpoints
        self.callbacks[TopicInfo.ID_PARAMETER_REQUEST] = self.handleParameterRequest
        self.callbacks[TopicInfo.ID_LOG] = self.handleLoggingRequest
        self.callbacks[TopicInfo.ID_TIME] = self.handleTimeRequest

        self.sub_ids=15
        self.shutdown = False

        # Reason about what local topics get exposed to remote bridges
        all_topics = rospy.get_param('/serial_bridge/share_all_topics', False)
        if all_topics:
            rospy.loginfo("share_all_topics set to True. All local topics will be accessible over bluetooth_bridge.")
            self.subscribeAllPublished()
        else:
            topic_whitelist = rospy.get_param('/serial_bridge/shared_topics_path', None)
            if topic_whitelist is None:
                rospy.logerr("Could not load topic whitelist under bluetooth_bridge/shared_topics.")

            rospy.loginfo("Attempting to retrieve shared topic list at " + topic_whitelist)
            with open(topic_whitelist, 'r') as stream:
                topic_whitelist = yaml.safe_load(stream)
                self.subscribeWhitelist(topic_whitelist)


        self.negotiateTopics()
        self.requestTopics()
        self.ever_synced = False
        self.lastsync = rospy.Time.now()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        #self.port.flushInput()
        # request topic sync
        if hasattr(self.port, 'write'):
            self.port.write("\xff" + self.protocol_ver + "\x00\x00\xff\x00\x00\xff")
        if hasattr(self.port, 'send'):
            self.port.send("\xff" + self.protocol_ver + "\x00\x00\xff\x00\x00\xff")


    def heartbeat(self):
        # send time request every timeout period, so the other side knows we are alive
        self.requestSyncTime()
        if not self.shutdown:
            threading.Timer(self.timeout,self.heartbeat).start()

    def shutdown_node(self):
        self.shutdown = True

    def set_port(self, port):
        self.port = port

    def close_port(self):
        if self.port is not None:
            self.port.close()
        self.port = None

    def run(self):
        """ Forward recieved messages to appropriate publisher. """
        data = ''
        # send time request every timeout period, so the other side knows we are alive
        threading.Timer(self.timeout,self.heartbeat).start()

        while not rospy.is_shutdown() and not self.shutdown:
            if self.port is None:
                rospy.sleep(0.05)
                continue
            if (rospy.Time.now() - self.lastsync).to_sec() > (self.timeout * 3):
                if (self.synced == True):
                    rospy.logerr("Lost sync with device, restarting...")
                else:
                    rospy.logerr("Unable to sync with other hosts on bridge. There may not be any remotes connected.")
                    self.synced = False
                    self.lastsync_lost = rospy.Time.now()
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "no sync with remote host")
                    self.negotiateTopics()
                    self.requestTopics()
                    self.lastsync = rospy.Time.now()

            # assumption: if flag is in read queue, entire message in queue
            # so only need to explicity check if port is open here
            flag = [0,0]
            flag[0] = self.read(1)
            if flag[0] is not None:
                flag[1] = self.read(1)
            else:
                self.shutdown = True
                continue

            if (flag[0] != '\xff'):
                continue
            if ( flag[1] != self.protocol_ver):
                if not self.ever_synced:
                    continue
                # clear the  serial port buffer
                #self.port.flushInput()
                self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client")
                rospy.logerr("Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client")
                protocol_ver_msgs = {'\xff': 'Rev 0 (rosserial 0.4 and earlier)', '\xfe': 'Rev 1 (rosserial 0.5+)', '\xfd': 'Some future rosserial version'}
                if (flag[1] in protocol_ver_msgs):
                    found_ver_msg = 'Protocol version of client is ' + protocol_ver_msgs[flag[1]]
                else:
                    found_ver_msg = "Protocol version of client is unrecognized"
                rospy.loginfo("%s, expected %s" % (found_ver_msg, protocol_ver_msgs[self.protocol_ver]))
                continue
            msg_len_bytes = None
            if hasattr(self.port, 'read'):
                msg_len_bytes = self.port.read(2)
            elif hasattr(self.port, 'recv'):
                msg_len_bytes = self.port.recv(2)

            if len(msg_len_bytes) != 2:
                continue

            msg_length, = struct.unpack("<h", msg_len_bytes)

            # checksum of msg_len
            msg_len_chk = None
            if hasattr(self.port, 'read'):
                msg_len_chk = self.port.read(1)
            elif hasattr(self.port, 'recv'):
                msg_len_chk = self.port.recv(1)

            msg_len_checksum = sum(map(ord, msg_len_bytes)) + ord(msg_len_chk)

            if msg_len_checksum%256 != 255:
                if self.ever_synced:
                    rospy.loginfo("wrong checksum for msg length, length %d" %(msg_length))
                    rospy.loginfo("chk is %d" %(ord(msg_len_chk)))
                    #self.port.flushInput()
                continue

            # topic id (2 bytes)
            topic_id_header = None
            if hasattr(self.port, 'read'):
                topic_id_header = self.port.read(2)
            elif hasattr(self.port, 'recv'):
                topic_id_header = self.port.recv(2)
            if len(topic_id_header)!=2:
                continue
            topic_id, = struct.unpack("<h", topic_id_header)

            msg = None
            if hasattr(self.port, 'read'):
                msg = self.port.read(msg_length)
            elif hasattr(self.port, 'recv'):
                msg = self.port.recv(msg_length)


            if (len(msg) != msg_length):
                self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Packet Failed :  Failed to read msg data")
                rospy.loginfo("Packet Failed :  Failed to read msg data")
                rospy.loginfo("msg len is %d",len(msg))
                continue

            # checksum for topic id and msg
            if hasattr(self.port, 'read'):
                chk = self.port.read(1)
            if hasattr(self.port, 'recv'):
                chk = self.port.recv(1)

            checksum = sum(map(ord, topic_id_header) ) + sum(map(ord, msg)) + ord(chk)
            if checksum%256 == 255:
                self.ever_synced = True
                self.synced = True
                try:
                    if self.compressed and msg_length>0:
                        msg = zlib.decompress(msg)
                    self.callbacks[topic_id](msg)
                except KeyError:
                    rospy.logwarn("Tried to publish before configured, topic id %d" % topic_id)
                    self.requestTopics()
                except:
                    rospy.logerr("Unexpected error: %s",sys.exc_info()[0])

                rospy.sleep(0.001)
        rospy.loginfo("Exiting")
        self.close_port()

    def setPublishSize(self, bytes):
        if self.buffer_out < 0:
            self.buffer_out = bytes
            rospy.loginfo("Note: publish buffer size is %d bytes" % self.buffer_out)

    def setSubscribeSize(self, bytes):
        if self.buffer_in < 0:
            self.buffer_in = bytes
            rospy.loginfo("Note: subscribe buffer size is %d bytes" % self.buffer_in)

    def subscribeWhitelist(self, topic_whitelist):
        msg = TopicInfo()
        msg.buffer_size = self.buffer_in
        blacklist_topics = ['/rosout', '/rosout_agg']
        if 'subscribers' in topic_whitelist.keys():
            topics = topic_whitelist['subscribers']
            for tup in topics.keys():
                topic_name = tup
                topic_type = topics[tup]

                if topic_name not in blacklist_topics:
                   msg.topic_id = self.sub_ids
                   self.sub_ids+=1
                   msg.topic_name = topic_name
                   msg.message_type = topic_type
                   pkg, msg_name = topic_type.split('/')
                   self.message = load_message(pkg, msg_name)
                   msg.md5sum = self.message._md5sum

                   sub = Subscriber(msg, self)
                   self.subscribers[msg.topic_name] = sub
                   self.setSubscribeSize(msg.buffer_size)
                   rospy.loginfo("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type) )


    def subscribeAllPublished(self):
        msg = TopicInfo()
        msg.buffer_size = self.buffer_in
        blacklist_topics = ['/rosout', '/rosout_agg']
        for tup in rospy.get_published_topics():
            topic_name = tup[0]
            topic_type = tup[1]

            if topic_name not in blacklist_topics:
               msg.topic_id = self.sub_ids
               self.sub_ids+=1
               msg.topic_name = topic_name
               msg.message_type = topic_type
               pkg, msg_name = topic_type.split('/')
               self.message = load_message(pkg, msg_name)
               msg.md5sum = self.message._md5sum

               sub = Subscriber(msg, self)
               self.subscribers[msg.topic_name] = sub
               self.setSubscribeSize(msg.buffer_size)
               rospy.loginfo("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type) )

    def setupPublisher(self, data):
        ''' Request to negotiate topics'''

        if len(data)==0:
            rospy.loginfo("Got request for topics!")
            self.requestSyncTime()
            self.negotiateTopics()
            return

        """ Register a new publisher. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            if msg.topic_name in self.publishers.keys():
                rospy.loginfo("Publisher exists on %s [%s]" % (msg.topic_name, msg.message_type) )
                return
            pub = Publisher(msg)
            self.publishers[msg.topic_name] = pub
            self.callbacks[msg.topic_id] = pub.handlePacket
            self.setPublishSize(msg.buffer_size)
            rospy.loginfo("Setup publisher on %s [%s]" % (msg.topic_name, msg.message_type) )
        except Exception as e:
            rospy.logerr("Creation of publisher failed: %s", e)

    def setupSubscriber(self, data):
        """ Register a new subscriber. """
        try:
            rospy.loginfo("Setting up a Subscriber!")
            msg = TopicInfo()
            msg.deserialize(data)
            if msg.topic_name in self.subscribers.keys():
                rospy.loginfo("Subscriber exists on %s [%s]" % (msg.topic_name, msg.message_type) )
                return
            sub = Subscriber(msg, self)
            self.subscribers[msg.topic_name] = sub
            self.setSubscribeSize(msg.buffer_size)
            rospy.loginfo("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type) )
        except Exception as e:
            rospy.logerr("Creation of subscriber failed: %s", e)

    def setupServiceServerPublisher(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except:
                srv = ServiceServer(msg, self)
                rospy.loginfo("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mres._md5sum == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' + srv.mres._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rospy.logerr("Creation of service server failed: %s", e)
    def setupServiceServerSubscriber(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except:
                srv = ServiceServer(msg, self)
                rospy.loginfo("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mreq._md5sum == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rospy.logerr("Creation of service server failed: %s", e)

    def setupServiceClientPublisher(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except:
                srv = ServiceClient(msg, self)
                rospy.loginfo("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mreq._md5sum == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rospy.logerr("Creation of service client failed: %s", e)

    def setupServiceClientSubscriber(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except:
                srv = ServiceClient(msg, self)
                rospy.loginfo("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mres._md5sum == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.mres._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rospy.logerr("Creation of service client failed: %s", e)

    def handleTimeRequest(self, data):
        """ Respond to device with system time. """
        now = rospy.Time.now()
        t = Time()
        t.deserialize(data)

        # if Time message empty, it's actually a request. Send own Time message to remote
        if t.time.data.secs == 0 and t.time.data.nsecs == 0:
            t = Time()
            t.hostname.data = socket.gethostname()
            t.time.data = rospy.Time.now()
            data_buffer = StringIO.StringIO()
            t.serialize(data_buffer)
            self.send( TopicInfo.ID_TIME, data_buffer.getvalue() )
            self.lastsync = rospy.Time.now()
        else:
            #Note: this captures both difference in ROS Time and time over the wire. As such
            # it may not be adequate for applications where tight synchronization is neeeded
            self.remote_time[t.hostname.data] = t.time.data
            self.remote_time_offset[t.hostname.data] = t.time.data - now #note: this results in a Duration

            #TODO Time offset is captured but not applied anywhere...

            rospy.logdebug("Time offset from remote " + t.hostname.data + ": " + str(self.remote_time_offset[t.hostname.data].to_sec()))
            self.lastsync = now
            pass


    def handleParameterRequest(self, data):
        """ Send parameters to device. Supports only simple datatypes and arrays of such. """
        req = RequestParamRequest()
        req.deserialize(data)
        resp = RequestParamResponse()
        try:
            param = rospy.get_param(req.name)
        except KeyError:
            rospy.logerr("Parameter %s does not exist"%req.name)
            return

        if param == None:
            rospy.logerr("Parameter %s does not exist"%req.name)
            return

        if (type(param) == dict):
            rospy.logerr("Cannot send param %s because it is a dictionary"%req.name)
            return
        if (type(param) != list):
            param = [param]
        #check to make sure that all parameters in list are same type
        t = type(param[0])
        for p in param:
            if t!= type(p):
                rospy.logerr('All Paramers in the list %s must be of the same type'%req.name)
                return
        if (t == int):
            resp.ints= param
        if (t == float):
            resp.floats=param
        if (t == str):
            resp.strings = param
        data_buffer = StringIO.StringIO()
        resp.serialize(data_buffer)
        self.send(TopicInfo.ID_PARAMETER_REQUEST, data_buffer.getvalue())

    def handleLoggingRequest(self, data):
        """ Forward logging information from serial device into ROS. """
        msg = Log()
        msg.deserialize(data)
        if (msg.level == Log.ROSDEBUG):
            rospy.logdebug(msg.msg)
        elif(msg.level== Log.INFO):
            rospy.loginfo(msg.msg)
        elif(msg.level== Log.WARN):
            rospy.logwarn(msg.msg)
        elif(msg.level== Log.ERROR):
            rospy.logerr(msg.msg)
        elif(msg.level==Log.FATAL):
            rospy.logfatal(msg.msg)

    def read(self, num_bytes):
        ret = None
        try:
            if hasattr(self.port, 'read'):
                ret  = self.port.read(num_bytes)
            elif hasattr(self.port, 'recv'):
                ret = self.port.recv(num_bytes)
        except:
            rospy.logwarn("Read from port failed. It's likely closed.")
        return ret

    def send(self, topic, msg):
        if self.compressed:
            msg = zlib.compress(msg,4)
        """ Send a message on a particular topic to the device. """
        with self.mutex:
            length = len(msg)
            if self.buffer_in > 0 and length > self.buffer_in:
                rospy.logerr("Message from ROS network dropped: message larger than buffer.")
                print(msg)
                return -1
            else:
                #modified frame : header(2 bytes) + msg_len(2 bytes) + msg_len_chk(1 byte) + topic_id(2 bytes) + msg(x bytes) + msg_topic_id_chk(1 byte)
                # second byte of header is protocol version
                msg_len_checksum = 255 - ( ((length&255) + (length>>8))%256 )
                msg_checksum = 255 - ( ((topic&255) + (topic>>8) + sum([ord(x) for x in msg]))%256 )
                data = "\xff" + self.protocol_ver  + chr(length&255) + chr(length>>8) + chr(msg_len_checksum) + chr(topic&255) + chr(topic>>8)
                data = data + msg + chr(msg_checksum)
                if hasattr(self.port, 'write'):
                    self.port.write(data)
                elif hasattr(self.port, 'send'):
                    try:
                        self.port.send(data)
                    except:
                        rospy.logwarn("Unable to send data over socket. Endpoint not connected. Socket likely closed. Resetting socket")
                        #TODO unregister any subscribers?
                        self.close_port()
                return length

    def sendDiagnostics(self, level, msg_text):
        msg = diagnostic_msgs.msg.DiagnosticArray()
        status = diagnostic_msgs.msg.DiagnosticStatus()
        status.name = "rosserial_python"
        msg.header.stamp = rospy.Time.now()
        msg.status.append(status)

        status.message = msg_text
        status.level = level

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[0].key="last sync"
        if self.lastsync.to_sec()>0:
            status.values[0].value=time.ctime(self.lastsync.to_sec())
        else:
            status.values[0].value="never"

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[1].key="last sync lost"
        status.values[1].value=time.ctime(self.lastsync_lost.to_sec())

        self.pub_diagnostics.publish(msg)

    def requestSyncTime(self):
        t = Time()
        data_buffer = StringIO.StringIO()
        t.serialize(data_buffer)
        self.send(TopicInfo.ID_TIME, data_buffer.getvalue())

    def negotiateTopics(self):
        #self.port.flushInput()
        outgoing_prefix = '/' + socket.gethostname()
        # publishers on this side require subscribers on the other, and viceversa

        ti = TopicInfo()
        """
        # This is meant to sync subscribers if publishers are set up locally.
        # This functionality should be implemented in the future, however it does not currently work as intended
        # Current practice is to set up subscribers locally which syncs Pubs on the remote
        for p_id in self.publishers.keys():
            p = self.publishers[p_id]
            ti.topic_id = p_id
            ti.topic_name = p.topic
            ti.message_type = p.message_type
            ti.md5sum = p.message._md5sum
            ti.buffer_size = self.buffer_out
            _buffer = StringIO.StringIO()
            ti.serialize(_buffer)
            self.send(TopicInfo.ID_SUBSCRIBER,_buffer.getvalue())
            time.sleep(0.01)
        """
        for s_name in self.subscribers.keys():
            s = self.subscribers[s_name]
            ti.topic_id = s.id
            ti.topic_name = outgoing_prefix+s.topic
            ti.message_type = s.message_type
            ti.md5sum = s.message._md5sum
            ti.buffer_size = self.buffer_in
            _buffer = StringIO.StringIO()
            ti.serialize(_buffer)
            self.send(TopicInfo.ID_PUBLISHER,_buffer.getvalue())
            time.sleep(0.01)

        # service clients on this side require service servers on the other, and viceversa
        for s_name in self.services.keys():
            s = self.services[s_name]
            ti.topic_id = s.topic_id
            ti.topic_name = s.topic
            ti.message_type = s.message_type
            ti.md5sum = s.message._md5sum
            ti.buffer_size = self.buffer_in

            _buffer = StringIO.StringIO()
            ti.serialize(_buffer)

            if s.__class__.__name__ == 'ServiceClient':
                self.send(TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_PUBLISHER,_buffer.getvalue())
                self.send(TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_SUBSCRIBER,_buffer.getvalue())
            if s.__class__.__name__ == 'ServiceServer':
                self.send(TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_PUBLISHER,_buffer.getvalue())
                self.send(TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_SUBSCRIBER,_buffer.getvalue())
            time.sleep(0.01)

        pass


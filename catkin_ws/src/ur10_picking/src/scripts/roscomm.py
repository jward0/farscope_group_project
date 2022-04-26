import rospy

class ServiceCaller:
    """
    Define service class
    This class is defined for each ROS service interaction
    The class can send a request to a service and process a response from a service
    :attribute servicename: string
        string name for the service as defined in the service definition
    :attribute service_class: class
        service class as defined in the service definition and imported from the relevent package
    """

    def __init__(self, servicename, service_class):
        self.servicename = servicename
        self.service_class = service_class
        rospy.wait_for_service(servicename)
        # rospy.loginfo("Connected to service:", servicename)

    def call(self, req):
        """
        Sends a service request to the service attributed with this class
        :param req: request to be sent to the class. The data type for this varies based on the service definition
        :return: returns the response from the service
        """
        rospy.wait_for_service(self.servicename)
        client = rospy.ServiceProxy(self.servicename, self.service_class)
        response = client(req)
        return response


class TopicReader:
    """
    Define topicreaderclass to read data from published ROS topics
    This class processes data from a ROS topic and returns it
    :attribute: topic_name - string name of the topic to which data will be published
    :attribute: data_class - data type / ROS data class for the published data
    """

    def __init__(self, topic_name, data_class):
        self.var = None
        self.topic_name = topic_name
        self.data_class = data_class
        rospy.Subscriber(self.topic_name, self.data_class, self.callback)
        rospy.sleep(1)
        rospy.loginfo("Reading topic {}...the following data has been read: {}".format(topic_name, self.var))

    def callback(self, data):
        """
        Callback function - this is called automatically from the readtopic() function below
        Sets the self.var variable to the data from the topic
        :param data: input autofilled through the rospy.Subscriber function in the readtopic() function below
        """
        self.var = data

    def read_topic(self):
        """
        :return: returns the data read from the topic
        """
        rospy.Subscriber(self.topic_name, self.data_class, self.callback)
        rospy.sleep(1)

        return self.var


class TopicWriter:
    """
    Define topicreader class to read data from published ROS topics
    This class processes data from a ROS topic and returns it
    :attribute: readname: name of the topic to subscribe and read from. Default to "none" if publishing only
    :attribute: writename: name of the topic to publish and write to. Default to "none" if subscribing only
    :attribute: read_data_class: data
    """

    def __init__(self, topic_name, data_class):
        self.var = None
        self.topic_name = topic_name
        self.data_class = data_class
        self.pub = rospy.Publisher(topic_name, self.data_class, queue_size=10)

    def write_topic(self, message):
        """
        Write to the topic that has been created
        :param message: Input data to be published to this topic
        """
        self.pub.publish(message)


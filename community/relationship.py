# TO DELETE #
# 
# 
# 
# 
# 
# somehow this is going to help define the way that people interact with each other.
# maybe one of these objects per group?

# needs to remember their relationship with others... like per person.

# a person should have like a state that they are set in 

# one instance per peson - NarrativeManager

# Create a narrative_config file for defining events that happen in time.  

# There needs to be a timer somewhere - one for the whole system.
# something built in to ros.
# Something aout ROS2 clock

# we get a global time... ah.  Have it in the launch file.
# Launch file gets current time.
# Then that is a parameter that is given to each initialised person.
# and then that is passed on to the NarrativeManager for that person when it is initialised.  


class Relationship():

    def __init__(self, person_1, person_2):
        pass
        # one relationship between 2 people!
        # could just have it as like a text file with important relationship history saved in it?
        # that history can be queried later for certain decision making tasks

    def update(sel):
        """
        
        
        """

    def tick_relationships(self):

    def controller(self):
        """
        Takes into account the current state of the person,
        and then checks for external events from the config file.
        Processes how these two will work together. 
        Push the person to a new state - emotional state.
        """
        # TODO need a 

        # controller could be different for different people.
        # depending on this person's personality traits.
        # e.g. they will be predisposed to react to certain things in certain ways.  

        # control inputs = mood, what the previous person said, what our relationship with them is, who else is in the group, where we are in the story etc.
        # system = the LLM
        # output state x = the words they say in response.
        # in the narrative manager we make the controller, so that we can give the LLM a very precise instruction and get out a cool answer.
        # BUT then the system becomes twofold as well I suppose.  Like if someone has been together 5 interactions and that is the lifetime of their relationship,
        # then the SYSTEM output state is BOTH the most recent thing said AND an action, like breakup.  But this action will only become apparent in the speech at the 
        # next iteraction -> only THIS part of the output state gets fed back to the contorller, you see?
        # and the part of the system responsible for actions like this goes in this class - requested by controller using request_state

        # The person is a state machine.  
        # This is one half of the system.
        # The other half of the system is an LLM.

        # When we are young... the noise on our sensors is REALLY high

        # nah... the behaviour tree is like the controller... it steps through time

        # we could leave the llm to decide things?  Maybe ask it yes or no?  

        # the controller is like the rest of the group?  Like the behaviour tree is the sytem, and left alone all the controller
        # does is tick time forwards by one step
        # but in a group, the controller must keep track of data from other and feed those into the behaviour tree...

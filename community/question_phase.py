import time, os, yaml
from ament_index_python.packages import get_package_share_directory

# Just a lil class for any other class that needs to keep track of the current question phase.
# The timer starts on class initialisation (so initialise it always at start of the system).

class GetQuestionPhase():

    def __init__(self) -> None:

        # Note the time the object was created, for checking against events
        self.initialise_time = time.time()

        # Get the path to the `events.yaml` file
        package_share_dir = get_package_share_directory('community')
        events_path = os.path.join(package_share_dir, 'config_files', 'events.yaml')

        with open(events_path, 'r') as file:
            events_yaml = yaml.safe_load(file)
        self.phase_data = events_yaml['phases']

    # Function to convert hours, minutes, and seconds to total seconds
    def convert_to_seconds(self, timestamp):
        hours = timestamp.get('hours', 0)
        minutes = timestamp.get('minutes', 0)
        seconds = timestamp.get('seconds', 0)
        total_seconds = (hours * 3600) + (minutes * 60) + seconds
        return total_seconds

    def get_question_phase(self):
        """
        Gets the current question phase.
        If 1, everyone is overly agreeable, helpful with each others' questions.
        If 2, we go to people starting to incorporate their own opinions on topics.
        If 3, we go to people getting more polarised and less helpful.  More emotive.
        If 4, we go to full anger, no listening to each other.  All selfish.
        If 5, we go to chaos, all talking over one another.
        """
        question_phase = 1 # We start in the overly agreeable phase

        time_now = time.time()
        elapsed_seconds = time_now - self.initialise_time
        # Check is the current time is past a given phase start time.
        for phase_id, phase in self.phase_data.items():
            timestamp = phase['timestamp']
            timestamp_seconds = self.convert_to_seconds(timestamp)
            if elapsed_seconds > timestamp_seconds:
                question_phase = phase_id

        return question_phase  
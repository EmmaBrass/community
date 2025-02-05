import time, os, yaml, wave
from ament_index_python.packages import get_package_share_directory

from piper.voice import PiperVoice

# Class of helper functions, for getting data from yaml config files, speech to bites, question phase etc.
# Timer starts on object creation.
class HelperFunctions():

    def __init__(self):

        # Note the time the object was created, for checking against events
        self.initialise_time = time.time()
        
        # Get the path to the `people.yaml` file and load people data
        package_share_dir = get_package_share_directory('community')
        people_path = os.path.join(package_share_dir, 'config_files', 'people.yaml')
        self.people_data = self.load_people(people_path)

        # Get the path to the `events.yaml` file and load event and phase data
        package_share_dir = get_package_share_directory('community')
        events_path = os.path.join(package_share_dir, 'config_files', 'events.yaml')
        self.events_data = self.load_events(events_path)
        self.phase_data = self.load_phases(events_path)

    def load_events(self, file_path):
        """ Load event data from the YAML file. """
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['events']
    
    def load_phases(self, file_path):
        """ Load phase data from the YAML file. """
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['phases']

    def load_people(self, file_path):
        """ Load people data from the YAML file. """
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['people']

    def text_to_speech_bytes(self, text, voice_id, id):
        """
        Convert some speech into .wav bytes for sending.

        :param text: The text to convert.
        :param voice_id: The voice_id to send.
        :param id: The group id or the person id, as a unique identifier for the file name (string or int).
        """
        try:
            voicedir = os.path.expanduser('~/Documents/piper/')  # Model directory
            model = voicedir + voice_id
            voice = PiperVoice.load(model)
            
            # Define output .wav file
            wav_file = f'audio_output_id_{id}.wav'
            print("MADE AUDIO OUTPUT FILE")
            
            # Open wave file in binary write mode
            with wave.open(wav_file, 'wb') as wav:
                # Set wave parameters (sample width, channels, frame rate)
                wav.setnchannels(1)  # Mono audio
                wav.setsampwidth(2)  # Typically 16-bit audio
                wav.setframerate(22050)  # Set frame rate to 22.05 kHz
                
                # Synthesize text and write audio frames
                voice.synthesize(text, wav)

            # Convert audio to bytes
            with open(wav_file, 'rb') as f:
                audio_data = f.read()
                audio_uint8 = list(audio_data)
                
            return audio_uint8

        except Exception as e:
            print(f"Error in text_to_speech: {e}")

    def get_voice_id(self, person_id):
        # Now initialize the person object using person attributes from config yaml file
        person_data = self.people_data.get(person_id, {})
        voice_id = person_data.get('voice_id', None)
        print(f"Voice ID is: {voice_id}")
        if voice_id == None:
            print("Error! Voice_id not found for this person_id")
        return str(voice_id)
    
    def get_color(self, person_id):
        # Now initialize the person object using person attributes from config yaml file
        person_data = self.people_data.get(person_id, {})
        color = person_data.get('color', None)
        print(f"Color is: {color}")
        if color == None:
            print("Error! Color not found for this person_id")
        return color

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
        If 6, we go to electronic static, all sounds overlapping.
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
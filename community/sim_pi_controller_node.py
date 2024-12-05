import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml, os
import curses

from community_interfaces.msg import SimPiPersonAssign
import community.configuration as config
from pynput import keyboard
from ament_index_python.packages import get_package_share_directory

class SimPiController(Node):
    """
    SimPiController node should be launched in a separate terminal to allow the curses interaction to work.
    Launch in terminal 2 using: ros2 run <your_package> sim_pi_controller_node
    """ 

    def __init__(self):
        super().__init__('pi_controller')

        # Get number of Pis in group one only (we only simulate one group on the computer)
        self.num_pis = len(config.GROUP_PI_ASSIGNMENTS.get(1).get('pi_ids'))
        if self.num_pis > 9:
            self.get_logger().error("Too many Pis in group 1 for simulation! Must be 9 or fewer.")

        # Keep track of which Pi has a person assigned (0 = no person)
        self.pi_status = [0] * self.num_pis
        self.selected_pi = None
        self.in_menu = False  # Track whether the curses menu is active

        # Load people from people.yaml
        self.people_ids, self.people_data = self.load_people()
        self.available_people = self.people_ids

        # Initialise publisher
        self.sim_pi_person_assign_publisher = self.create_publisher(SimPiPersonAssign, 'sim_pi_person_assign', 10)

        # Start the keyboard listener
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        # Display initial instructions
        self.display_instructions()

    def load_people(self):
        try:
            # Load people.yaml
            package_share_dir = get_package_share_directory('community')
            people_path = os.path.join(package_share_dir, 'config_files', 'people.yaml')
            with open(people_path, 'r') as file:
                data = yaml.safe_load(file)
                people_data = data['people']
                people_ids = list(data['people'].keys())
                self.get_logger().info(f"Loaded people: {people_ids}")
                return [int(pid) for pid in people_ids], people_data
        except Exception as e:
            self.get_logger().error(f"Failed to load people.yaml: {e}")
            return []

    def display_instructions(self):
        # Display current instructions and status
        self.get_logger().info("\nPlease press a digit key from 1 to {} to access a Pi.".format(self.num_pis))
        self.get_logger().info("Current Pi assignments:")
        for idx, person_id in enumerate(self.pi_status):
            self.get_logger().info(f"  Pi {idx + 1}: {'None' if person_id == 0 else person_id}")

    def on_press(self, key):
        if self.in_menu:
            return  # Ignore keypresses while in the curses menu

        try:
            # Check if a number key is pressed to select a Pi
            if key.char.isdigit():
                pi_number = int(key.char)
                if 1 <= pi_number <= self.num_pis:
                    self.in_menu = True
                    self.selected_pi = pi_number
                    self.get_logger().info(f'Selected Pi {pi_number}')
                    self.toggle_person_for_selected_pi()
                    self.display_instructions()  # Refresh instructions with updated status
                    self.in_menu = False

        except AttributeError:
            self.get_logger().warning("Pressed a non-digit key.")
            pass

    def toggle_person_for_selected_pi(self):
        if self.selected_pi is None:
            return

        # Check if there is a person already on this Pi
        pi_index = self.selected_pi - 1
        person_there = self.pi_status[pi_index]
        if person_there == 0:
            # No person, add one from available_people
            person_id = self.select_person_from_menu()
            if person_id is not None:
                self.get_logger().info(f'Adding person {person_id} to Pi {self.selected_pi}')
                self.pi_status[pi_index] = person_id
                self.available_people.remove(person_id)  # Remove from available people
                self.send_update_to_pi(person_id)
        else:
            # Person already there, remove them
            self.get_logger().info(f'Removing person {person_there} from Pi {self.selected_pi}')
            self.pi_status[pi_index] = 0
            self.available_people.append(person_there)  # Make available again
            self.send_update_to_pi(0)

    def select_person_from_menu(self):
        """
        Use a curses-based interface to select a person from available_people.
        """
        def draw_menu(stdscr):
            # Clear screen and display menu
            stdscr.clear()
            curses.curs_set(0)  # Hide cursor

            current_row = 0
            while True:
                stdscr.clear()
                stdscr.addstr(0, 0, "Select a person with the arrow keys and press Enter:")

                # Display the available people in the menu
                for idx, person_id in enumerate(self.available_people):
                    if idx == current_row:
                        stdscr.addstr(idx + 1, 0, f"> {person_id}", curses.A_REVERSE)
                    else:
                        stdscr.addstr(idx + 1, 0, f"  {person_id}")

                # Wait for user input
                key = stdscr.getch()

                if key == curses.KEY_UP and current_row > 0:
                    current_row -= 1
                elif key == curses.KEY_DOWN and current_row < len(self.available_people) - 1:
                    current_row += 1
                elif key == curses.KEY_ENTER or key in [10, 13]:
                    return self.available_people[current_row]

                stdscr.refresh()

        # Activate menu flag and open curses wrapper
        selected_person = curses.wrapper(draw_menu)
        self.get_logger().info(f'Selected person: {selected_person}')
        return selected_person

    def send_update_to_pi(self, person_id):
        if self.selected_pi is None:
            self.get_logger().warn("No selected pi!")
            return

        msg = SimPiPersonAssign()
        msg.pi_id = self.selected_pi
        msg.person_id = person_id
        msg.voice_id = self.get_voice_id(person_id)
        for i in range(5):
            self.sim_pi_person_assign_publisher.publish(msg)

    def get_voice_id(self, person_id):
        # Now initialize the person object using person attributes from config yaml file
        person_data = self.people_data.get(person_id, {})
        voice_id = person_data.get('voice_id', None)
        self.get_logger().info(f"Voice ID is: {voice_id}")
        if voice_id == None:
            self.get_logger().info("Error! Voice_id not found for this person_id")
        return str(voice_id)


def main(args=None):
    rclpy.init(args=args)
    sim_pi_controller_node = SimPiController()
    rclpy.spin(sim_pi_controller_node)
    sim_pi_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

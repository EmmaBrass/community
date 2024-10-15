import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import curses

from community_interfaces.msg import SimPiPersonAssign

import community.configuration as config

from pynput import keyboard

class SimPiController(Node):
    """
    SimPiController node should be launched in a seperate terminal to the 
    rest of the system, to allow the curses interaction to work.
    Launch in terminal 2 using: ros2 run <your_package> sim_pi_controller_node or something like that...
    """

    def __init__(self):
        super().__init__('pi_controller')

        # Get number of pis in group one only (we only simulate one group on the computer)
        self.num_pis = len(config.GROUP_PI_ASSIGNMENTS.get(1).get('pi_ids'))
        # Throw an error if more tha 9 pis (we only simulate up to 9 using one numerical key for each)
        if self.num_pis > 9:
            self.get_logger().error("Too many Pis in group 1 for simulation!  Must be 9 or fewer.")

        # Keep track of which Pi has a person assigned (0 = no person)
        self.pi_status = [0]*self.num_pis  # Up to 9 Pis
        self.selected_pi = None

        # Initialise publisher
        self.sim_pi_person_assign_publisher = self.create_publisher(SimPiPersonAssign, f'sim_pi_person_assign', 10)

        # Start the keyboard listener
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        # Load people from people.yaml
        self.all_people = self.load_people_ids()
        self.available_people = self.all_people
        
    def load_people_ids(self):
        """
        Load people IDs from the people.yaml file and return a list of IDs.
        """
        try:
            with open('people.yaml', 'r') as file:
                data = yaml.safe_load(file)
                people_ids = list(data['people'].keys())  # Get the IDs (keys) from the people dictionary
                self.get_logger().info(f"Loaded people: {people_ids}")
                return [int(pid) for pid in people_ids]  # Convert to integers
        except Exception as e:
            self.get_logger().error(f"Failed to load people.yaml: {e}")
            return []

    def on_press(self, key):
        try:
            # Check if a number key is pressed to select a Pi
            if key.char.isdigit():
                pi_number = int(key.char)
                if 1 <= pi_number <= 9:
                    self.selected_pi = pi_number
                    self.get_logger().info(f'Selected Pi {pi_number}')

                    # Toggle adding/removing a person
                    self.toggle_person_for_selected_pi()

        except AttributeError:
            self.get_logger().warning("Pressed a non-digit key?")
            # Handle special keys like 'Key.space'
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
                self.available_people.remove(person_id) # They are now not available to be put on another pi
                self.send_update_to_pi(person_id)
        else:
            # Person already there, remove them
            self.get_logger().info(f'Removing person {person_there} from Pi {self.selected_pi}')
            self.pi_status[pi_index] = 0
            self.available_people.append(person_there) # They are now available to be put on another pi
            self.send_update_to_pi(0)

    def select_person_from_menu(self):
        """
        Use a curses-based interface to select a person from available_people.
        """
        def draw_menu(stdscr):
            # Clear screen
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
                    # Return the selected person_id when Enter is pressed
                    return self.available_people[current_row]

                stdscr.refresh()

        # Initialize curses
        selected_person = curses.wrapper(draw_menu)
        self.get_logger().info(f'Selected person: {selected_person}')
        return selected_person

    def send_update_to_pi(self, pi_id, person_id):
        if self.selected_pi is None:
            return
        
        msg = SimPiPersonAssign()
        msg.pi_id = self.selected_pi
        msg.person_id = person_id
        # Send the update to the corresponding PiNode
        self.sim_pi_person_assign_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    sim_pi_controller_node = SimPiController()

    rclpy.spin(sim_pi_controller_node)

    # Cleanup
    sim_pi_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
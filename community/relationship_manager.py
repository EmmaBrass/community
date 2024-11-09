import random, yaml, os, copy

import community.configuration as config
from community.relationships_state_machine import RelationshipMachine

from ament_index_python.packages import get_package_share_directory

class RelationshipManager:
    def __init__(self):
        self.relationships = {}
        self.tick_counter = [0]*len(config.GROUP_PI_ASSIGNMENTS)
        self.history = {}  # This will store all tick states by ID and group

        # Get the path to the 'people.yml' file
        package_share_dir = get_package_share_directory('community')
        people_path = os.path.join(package_share_dir, 'config_files', 'people.yaml')
        # Load person_ids from yaml file
        self.person_ids = list(self.load_people(people_path).keys())

        # Get the path to the `relationships.yaml` file
        package_share_dir = get_package_share_directory('community')
        relationships_path = os.path.join(package_share_dir, 'config_files', 'relationships.yaml')
        self.relationships_data = self.load_relationships(relationships_path)

        self.init_relationships()

    def load_people(self, file_path):
        """ Load people data from the YAML file. """
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['people']

    def load_relationships(self, file_path):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['relationships']

    def init_relationships(self):
        """
        Initialise the pairwise relationships matrix.
        """
        
        # Number of people
        num_people = len(self.person_ids)

        # Initialize an empty 2D array for relationships, with each entry as None
        self.relationships_matrix = [[None for _ in range(num_people)] for _ in range(num_people)]

        # Initialize the relationships matrix with fields: 'state_machine', 'interactions_left', 'action'
        for i in range(num_people):
            for j in range(i + 1, num_people):
                relationship_machine = None  # A state machine in a certain state that can be queried
                # Store the same reference in both [i][j] and [j][i]
                self.relationships_matrix[i][j] = relationship_machine
                self.relationships_matrix[j][i] = relationship_machine

        # # Example: access a relationship between person 1 and person 2
        # person_index_1 = 0  # index of the first person in the list
        # person_index_2 = 1  # index of the second person in the list

        # relationship_machine = self.relationships_matrix[person_index_1][person_index_2]
        # print(f"Initial relationship machine between person 1 and person 2: {relationship_machine}")

        # # Update the state machine, interactions left, and action for this relationship
        # relationship_machine = 'dating'

        # # Since both [i][j] and [j][i] point to the same object, the update applies to both
        # print(f"Updated relationship between person 1 and person 2: {self.relationships_matrix[person_index_1][person_index_2]}")
        # print(f"Relationship between person 2 and person 1 (should be identical): {self.relationships_matrix[person_index_2][person_index_1]}")

    def create_tick_snapshot(self, group_id, group_members):
        """
        Create a snapshot of relationship_machine for a particular group_id.
        Only stores relationships where both members are in the group_members list.
        """
        # Ensure history has an entry for the group_id
        if group_id not in self.history:
            self.history[group_id] = {}

        # Filter relationships that only involve members of the group
        filtered_relationships = {}

        # Create a snapshot of the relationships that involve group members
        for i in range(len(group_members)):
            for j in range(i + 1, len(group_members)):
                member_a = group_members[i]
                member_b = group_members[j]

                if member_a in self.person_ids and member_b in self.person_ids:
                    idx_a = self.person_ids.index(member_a)
                    idx_b = self.person_ids.index(member_b)

                    # Make a deepcopy of the relationship machine at the location of member_a and member_b in the matrix
                    relationship_machine = self.relationships_matrix[idx_a][idx_b]
                    filtered_relationships[(member_a, member_b)] = copy.deepcopy(relationship_machine)

        # Store the snapshot for this tick and group
        snapshot = {
            'tick_id': self.tick_counter[group_id - 1],
            'relationships': filtered_relationships
        }
        
        self.history[group_id][self.tick_counter[group_id - 1]] = snapshot
        print(f"Tick {self.tick_counter[group_id - 1]} recorded for group {group_id}.")
        
        # Increment tick counter for this group
        self.tick_counter[group_id - 1] += 1

    def rewind_to_tick(self, group_id, group_members, tick_id):
        """
        For all pairwise relationships where both people are current members 
        of the group with group_id, rewind these relationships to the given tick_id.
        """
        if group_id in self.history and tick_id in self.history[group_id]:
            snapshot = self.history[group_id][tick_id]

            # Only restore relationships between people in the specified group
            for (member_a, member_b), value in snapshot['relationships'].items():
                if member_a in group_members and member_b in group_members:
                    idx_a = self.person_ids.index(member_a)
                    idx_b = self.person_ids.index(member_b)

                    # Restore the relationship data for these two people
                    self.relationships_matrix[idx_a][idx_b] = value
                    self.relationships_matrix[idx_b][idx_a] = value  # Since both refer to the same relationship
            
            print(f"Rewound relationships in group {group_id} to tick {tick_id}.")
            self.tick_counter[group_id - 1] = tick_id
            return True
        else:
            print(f"Tick {tick_id} not found in history for group {group_id}.")
            return False

    def tick_get_relationship(self, person_a, person_b, group_id, group_members):
        """
        Tick the relationship between two people on by one. 
        Return a result with info about the state of the relationship and any actions.

        :param person_a: First person (person who will speak)
        :param person_b: Second person (person who is spoken to)
        :param group_id: The group id of the group these two people are in
        :param group_members: ALL people in the group (including the speakers)

        :return result: Dict with info about the new relationship
        :return tick_id: Tick id for the relationship iteration number
        """
        # Get indices for both people, and ensure idx_a < idx_b
        idx_a, idx_b = sorted([self.person_ids.index(person_a), self.person_ids.index(person_b)])

        relationship_machine = self.relationships_matrix[idx_a][idx_b]
        # If no relationship, maybe start one
        if relationship_machine == None:
            state_machine = self.setup_relationship_machine()
            self.relationships_matrix[idx_a][idx_b] = state_machine
            self.relationships_matrix[idx_b][idx_a] = state_machine
            result = {
                'state_changed': True,
                'from_state': None,
                'to_state': state_machine.get_current_state(),
                'transition_description': None,
                'action': None
            }
        else:
            # Tick the existing state machine 
            result = relationship_machine.tick()
            if relationship_machine.get_current_state() == 'no_relationship':
                # If state = 'no_relationship' then replace with None.
                self.relationships_matrix[idx_a][idx_b] = None
                self.relationships_matrix[idx_b][idx_a] = None
            else:
                self.relationships_matrix[idx_a][idx_b] = relationship_machine
                self.relationships_matrix[idx_b][idx_a] = relationship_machine
                
        self.create_tick_snapshot(group_id, group_members) # TODO tick snapshot should happen before or after the actual tick?
        # Get current tick_id for this group
        tick_id = self.tick_counter[group_id - 1]

        return result, tick_id

    def choose_relationship_type(self):
        """ Decide which relationship type to enter based on 'chance_to_start' probabilities. """
        
        romantic_chance = self.relationships_data['romantic_relationship'].get('chance_to_start', 0)
        friendship_chance = self.relationships_data['friendship_relationship'].get('chance_to_start', 0)

        total_chance = romantic_chance + friendship_chance
        if total_chance == 0:
            return None  # No relationship should start

        # Normalize chances and pick based on probabilities
        pick = random.uniform(0, total_chance)
        if pick <= romantic_chance:
            print("STARTING ROMANTIC RELATIONSHP")
            return 'romantic_relationship'
        else:
            print("STARTING FRIENDSHIP RELATIONSHP")
            return 'friendship_relationship'

    def setup_relationship_machine(self):
        """
        See if a relationship will start and choose the type.
        """

        # Decide which relationship to enter based on the chances
        chosen_relationship_type = self.choose_relationship_type()
        
        if chosen_relationship_type is None:
            print("No relationship started based on the probabilities.")
            return None

        # Create the state machine for the chosen relationship type
        state_machine = RelationshipMachine(chosen_relationship_type)
        
        # Start at the initial state
        initial_state = state_machine.get_current_state()

        # If the state is 'no_relationship', replace the state machine with None
        if initial_state == 'no_relationship':
            state_machine = None
        else:
            # Otherwise, start the state and assign interaction count
            state_machine.start_state(initial_state)
        return state_machine

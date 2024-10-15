import random
from transitions import Machine
import yaml

import community.configuration as config


class RelationshipManager:
    def __init__(self):
        self.relationships = {}
        self.tick_counter = []*len(config.GROUP_PI_ASSIGNMENTS)
        self.history = {}  # This will store all tick states by ID and group

        # Load the YAML file
        with open('path_to_people_yaml.yaml', 'r') as file:
            people_data = yaml.safe_load(file)
        # Extract person IDs
        self.person_ids = list(people_data['people'].keys())

        self.init_relationships()

    def init_relationships(self):
        """
        Initialise the pairwise relationships matrix.
        """
        
        # Number of people
        num_people = len(self.person_ids)

        # Initialize an empty 2D array for relationships, with each entry as None
        relationships_matrix = [[None for _ in range(num_people)] for _ in range(num_people)]

        # Initialize the relationships matrix with fields: 'state_machine', 'interactions_left', 'action'
        for i in range(num_people):
            for j in range(i + 1, num_people):
                relationship_data = {
                    'state': None,  # Initially no state machine
                    'interactions_done_in_state': None,
                    'interactions_left_for_state': None,  # Initially no interactions left
                    'action': None  # Initially no action
                    'num_interactions_for_action': None # How many interactions based on that action (clear action after 2 interactions)
                }
                # Store the same reference in both [i][j] and [j][i]
                relationships_matrix[i][j] = relationship_data
                relationships_matrix[j][i] = relationship_data

        # Example: access a relationship between person 1 and person 2
        person_index_1 = 0  # index of the first person in the list
        person_index_2 = 1  # index of the second person in the list

        relationship = relationships_matrix[person_index_1][person_index_2]
        print(f"Initial relationship between person 1 and person 2: {relationship}")

        # Update the state machine, interactions left, and action for this relationship
        relationship['state'] = 'dating'
        relationship['interactions_left'] = 10
        relationship['action'] = 'buy a puppy'

        # Since both [i][j] and [j][i] point to the same object, the update applies to both
        print(f"Updated relationship between person 1 and person 2: {relationships_matrix[person_index_1][person_index_2]}")
        print(f"Relationship between person 2 and person 1 (should be identical): {relationships_matrix[person_index_2][person_index_1]}")

    def create_tick_snapshot(self, group_id, group_members):
        """
        Create a snapshot of relationships for a particular group_id.
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

                    # Store the relationship between member_a and member_b
                    filtered_relationships[(member_a, member_b)] = self.relationships_matrix[idx_a][idx_b].copy()

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
        else:
            print(f"Tick {tick_id} not found in history for group {group_id}.")


    def tick_get_relationship(self, person_a, person_b, group_members):
        """
        Tick the relationship between two people on by one. 
        Return the current state of the relationship.
        """
        # Get the index from the initial list of people in the config yaml file, for standardisation
        idx_a = self.person_ids.index(person_a)
        idx_b = self.person_ids.index(person_b)

        # Should be no action when two people have just entered a new relationship state?
        # An action is only used for 2 interactions? or like max 4? could define in 
        # Hmmm no action should be only one interaction.  Something one person does.  The other's response is left to the LLM!

        # stuff
        self.create_tick_snapshot(self, group_id, group_members) # TODO tick snapshot should happen before or after the actual tick?





#########################

    def start_relationship(self, person_a, person_b):
        key = (person_a, person_b) if person_a < person_b else (person_b, person_a)

        # Define the state machine for the relationship
        states = ['meeting', 'getting_closer', 'committed', 'breakup']
        machine = Machine(model=self, states=states, initial='meeting')
        machine.add_transition('get_closer', 'meeting', 'getting_closer')
        machine.add_transition('commit', 'getting_closer', 'committed')
        machine.add_transition('break_up', 'committed', 'breakup')

        interactions_left = random.randint(5, 20)

        # Store the relationship in the manager
        self.relationships[key] = {
            'state_machine': machine,
            'interactions_left': interactions_left,
            'action': None
        }

        print(f"Relationship started between {person_a} and {person_b}.")

    def record_interaction(self, person_a, person_b):
        key = (person_a, person_b) if person_a < person_b else (person_b, person_a)

        relationship = self.relationships.get(key)
        if relationship and relationship['state_machine'] and relationship['interactions_left'] > 0:
            relationship['interactions_left'] -= 1
            self.maybe_trigger_action(person_a, person_b)
            if relationship['interactions_left'] <= 0:
                self.end_relationship(person_a, person_b)
        self.create_tick_snapshot()

    def maybe_trigger_action(self, person_a, person_b):
        key = (person_a, person_b) if person_a < person_b else (person_b, person_a)
        relationship = self.relationships[key]

        if relationship['state_machine']:
            if relationship['state_machine'].state in ['getting_closer', 'committed']:
                if random.random() < 0.05:
                    relationship['action'] = 'buy a puppy'
                    print(f"{person_a} and {person_b} bought a puppy!")
                elif random.random() < 0.10:
                    relationship['action'] = 'have an argument'
                    print(f"{person_a} and {person_b} had an argument!")

    def end_relationship(self, person_a, person_b):
        key = (person_a, person_b) if person_a < person_b else (person_b, person_a)
        relationship = self.relationships[key]

        if relationship['state_machine']:
            relationship['state_machine'].break_up()
            print(f"Relationship between {person_a} and {person_b} has ended.")
        self.create_tick_snapshot()

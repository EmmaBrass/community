import random, os, yaml
from transitions import Machine

from ament_index_python.packages import get_package_share_directory

# Load the YAML configuration
def load_relationships_from_yaml():
    # Get the path to the `people.yaml` file
    package_share_dir = get_package_share_directory('community')
    relationships_path = os.path.join(package_share_dir, 'config_files', 'relationships.yaml')
    with open(relationships_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['relationships']

# Generic relationship model class
class RelationshipModel:
    def __init__(self):
        self.interactions_left = 0  # Track interactions left in current state
        self.interactions_in_state = 0  # Track how many interactions we've been in the state
    
    def transition_with_probability(self, prob):
        """ Decide whether to transition based on probability """
        return random.random() < prob

    def random_transition(self, transitions):
        """ Choose a random transition based on the probabilities """
        rand = random.random()
        cumulative_prob = 0
        for state, details in transitions.items():
            cumulative_prob += details['probability']
            if rand < cumulative_prob:
                return state, details['description']
        return None

    def random_action(self, actions):
        """ Randomly determine if an action occurs based on action probabilities """
        rand = random.random()
        cumulative_prob = 0
        for _, details in actions.items():
            prob = details['probability']
            cumulative_prob += prob
            if rand < cumulative_prob:
                return details['description']  # Return the action description
        return None

# RelationshipMachine class to handle machine creation and transitions
class RelationshipMachine:
    def __init__(self, relationship_type):
        # Load the relationship config from the YAML file
        self.relationship_config = load_relationships_from_yaml()
        self.relationship_type = relationship_type

        # Create the model that will store state
        self.model = RelationshipModel()

        # Create the machine (state machine) for the given relationship type (romantic, friendship, etc.)
        self.machine = self.create_machine(self.model, relationship_type)
        
        # Keep track of interaction bounds and actions from the YAML
        self.interaction_bounds = self.get_interaction_bounds(relationship_type)
        self.actions = self.get_actions(relationship_type)

        # Actions should be left for an interation (to allow response) before another happens
        self.active_action = False

    def create_machine(self, model, relationship_type):
        """ Create the state machine based on the relationship configuration """
        states = list(self.relationship_config[relationship_type]['states'].keys())
        transitions = []

        # Dynamically add transitions based on the YAML file
        for state, details in self.relationship_config[relationship_type]['states'].items():
            for next_state, transition_info in details['transitions'].items():
                transitions.append({
                    'trigger': f'{state}_to_{next_state}',
                    'source': state,
                    'dest': next_state,
                    'conditions': f'probability_{state}_to_{next_state}'
                })

                # Define the condition functions for each transition based on the YAML probabilities
                probability_func_name = f'probability_{state}_to_{next_state}'
                probability = transition_info['probability']
                setattr(model, probability_func_name, lambda prob=probability: model.transition_with_probability(prob))

        # Create and return the state machine
        machine = Machine(model=model, states=states, transitions=transitions, initial=states[0])
        return machine

    def get_interaction_bounds(self, relationship_type):
        """ Get the interaction limits (min and max) for each state from the YAML config """
        interaction_bounds = {}
        for state, details in self.relationship_config[relationship_type]['states'].items():
            interaction_bounds[state] = {
                'min': details['min_interactions'],
                'max': details['max_interactions']
            }
        return interaction_bounds
    
    def get_actions(self, relationship_type):
        """ Get the action probabilities and descriptions for each state from the YAML config """
        actions = {}
        for state, details in self.relationship_config[relationship_type]['states'].items():
            actions[state] = details.get('actions', {})  # Get the full action details (name, description, probability)
        return actions

    def start_state(self, state):
        """ 
        Reset any active actions.
        Set the number of interactions for the current state based on its bounds. 
        """
        self.active_action = False
        bounds = self.interaction_bounds.get(state)
        if bounds:
            self.model.interactions_left = random.randint(bounds['min'], bounds['max'])
            self.model.interactions_in_state = 0  # Reset interactions in state
            print(f"Entering state '{state}' with {self.model.interactions_left} interactions left.")
        else:
            print(f"No interaction bounds found for state '{state}'.")
    
    def check_for_action(self):
        """ Check if an action occurs based on the action probabilities for the current state """
        current_state = self.get_current_state()
        if self.model.interactions_in_state > 2 and self.model.interactions_left > 1 and self.active_action == False:  # Only check after 2 interactions in the state, and more than 1 interaction left
            actions = self.actions.get(current_state, {})
            if actions:
                action_description = self.model.random_action(actions)
                if action_description:
                    print(f"Action occurred: {action_description}")
                    self.active_action = True
                    return action_description
        self.active_action = False
        return None

    def tick(self):
        """
        Progress one interaction, checking if we should move to another state or if an action occurs.
        Returns information about state transitions and actions.
        """
        result = {
            'state_changed': False,
            'from_state': None,
            'to_state': None,
            'transition_description': None,
            'action': None
        }

        # Update interaction counters
        self.model.interactions_left -= 1
        self.model.interactions_in_state += 1
        print(f"Interactions left in current state: {self.model.interactions_left}")

        # Check for action if applicable
        action_description = self.check_for_action()
        if action_description:
            result['action'] = action_description

        # If interactions in the current state are exhausted, move to the next state
        if self.model.interactions_left <= 0:
            current_state = self.get_current_state()
            possible_transitions = self.get_possible_transitions(current_state)

            if possible_transitions:
                next_state, transition_description = self.model.random_transition(possible_transitions)
                if next_state:
                    result['state_changed'] = True
                    result['from_state'] = current_state
                    result['to_state'] = next_state
                    result['transition_description'] = transition_description
                    trigger_name = f"{current_state}_to_{next_state}"
                    self.trigger_transition(trigger_name)
                    print(f"State changed from '{current_state}' to '{next_state}'")
                    self.start_state(next_state)  # Reset interactions for the new state

        return result

    def get_possible_transitions(self, state):
        """ Return the possible transitions for a given state with their probabilities """
        state_details = self.relationship_config.get(str(self.relationship_type), {}).get('states', {}).get(state, {})
        return {next_state: details for next_state, details in state_details.get('transitions', {}).items()}

    def     get_current_state(self):
        """ Get the current state of the relationship """
        return self.model.state

    def trigger_transition(self, trigger_name):
        """ Trigger a transition if it exists """
        try:
            trigger_method = getattr(self.model, trigger_name)
            trigger_method()  # Call the method to trigger the transition
        except AttributeError:
            print(f"Transition '{trigger_name}' does not exist.")

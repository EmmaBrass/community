
# Not currently in use...
# Could use for a better abstraction of the interactions memory for a person

class Interactions():

    def __init__(self):
        self.interactions = []

    def add_interaction(self, interaction: dict):
        """
        Add an interaction to the interactions database.  
        """
        self.interactions.append(interaction)
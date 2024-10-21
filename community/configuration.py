
########## For configuring the people in the system, and system setup ##########

# TODO
# A human writer should be able to give a long rambly text description of how people know each other and then
# an LLM will take that rambly description and parse it into the relationship.txt files
# Then a seperate rambly text file for events that happen during time

# 1. Describe the existing relationships between all the personas in the system.
# (This one used to initialise relationship xml/yaml files)
# 2. Describe how relationships COULD evolve over the course of the narrative timeline.  Or like what would happen if two people meet.
# (This one gets parsed into the events timeline ... ?)
# (Details of what is likely to happen between two people IF they meet)
# 3. Describe events that happen at given times that affect all conversations
# (Details on what % of the population hear about the event.  Give specific details on timings.  






NUM_GROUPS = 1

GROUP_PI_ASSIGNMENTS = {
    1 : {'pi_ids': [1,2]}
}

#     {'group_id': 2, 'pi_ids': [3,4,5]},
#     {'group_id': 3, 'pi_ids': [6]},
#     {'group_id': 4, 'pi_ids': [7,8,9,10,11]}
# ]

# Max number of back and forth exchanges between just two people before someone else is asked to speak.
BACK_AND_FORTH_MAX = 8

# Probability that a back and forth exchange get interrupted before BACK_AND_FORTH_MAX
INTERRUPT_PERCENT = 20

# How often (as a percentage of the time) a person directs a message at someone else precise, rather than just an open comment
DIRECT_PERCENT = 40

# How many pieces of text for future speech should be stored at any time, so that we don't needlessly over-query the GPT
MAX_SPEAK_LIST_LEN = 6








# self.pi_person_assignment = [
#     {'group_id': 1, 'members': [
#         {'pi_id': 1, 'person_id': 5},
#         {'pi_id': 2, 'person_id': 6},
#         {'pi_id': 3, 'person_id': 3},
#     ]},
#     {'group_id': 2, 'members': [
#         {'pi_id': 4, 'person_id': 4},
#         {'pi_id': 5, 'person_id': 7}
#     ]},
#     {'group_id': 2, 'members': [
#         {'pi_id': 6, 'person_id': 1}
#     ]},
#     {'group_id': 2, 'members': [
#         {'pi_id': 7, 'person_id': 2},
#         {'pi_id': 8, 'person_id': 8},
#         {'pi_id': 9, 'person_id': 10},
#         {'pi_id': 10, 'person_id': 9},
#         {'pi_id': 11, 'person_id': 11}
#     ]},
# ]

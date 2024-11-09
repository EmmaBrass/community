import unittest
from community.relationship_manager import RelationshipManager

class TestRelationshipManager(unittest.TestCase):
    """
    The basic class that inherits unittest.TestCase
    """
    relationship_manager = RelationshipManager()
    group_id = 1

    def test_1_load_people(self):
        print(f"{self.relationship_manager.person_ids = }")
    
    def test_2_load_relationships(self):
        print(f"{self.relationship_manager.relationships_data = }")
    
    def test_3_init_relationships(self):
        print(f"{self.relationship_manager.relationships_matrix = }")
    
    def test_4_tick_get_relationship(self):

        idx_a = self.relationship_manager.person_ids.index(117227880)
        idx_b = self.relationship_manager.person_ids.index(5299113)
        print(f"{self.relationship_manager.relationships_matrix[idx_a][idx_b]=}")
        print(f"{self.relationship_manager.relationships_matrix[idx_b][idx_a]=}")

        # First tick
        print("FIRST TICK")
        result, tick_id = self.relationship_manager.tick_get_relationship(117227880, 5299113, self.group_id, [5299113, 117227880, 17223924946])
        self.assertIsInstance(tick_id, int, "tick_id is not of type int")
        self.assertIsInstance(result, dict, "result is not of type dict")
        self.assertTrue(result['state_changed'] is True)
        self.assertTrue(result['from_state'] is None)
        self.assertIsInstance(result['to_state'], str)
        self.assertTrue(result['transition_description'] is None)
        self.assertTrue(result['action'] is None)

        print(f"{self.relationship_manager.relationships_matrix[idx_a][idx_b]=}")
        print(f"{self.relationship_manager.relationships_matrix[idx_b][idx_a]=}")

        # Second tick
        print("SECOND TICK")
        result, tick_id = self.relationship_manager.tick_get_relationship(117227880, 5299113, self.group_id, [5299113, 117227880, 17223924946])
        self.assertIsInstance(tick_id, int, "tick_id is not of type int")
        self.assertIsInstance(result, dict, "result is not of type dict")
        self.assertIsInstance(result['state_changed'], bool)
        self.assertTrue(isinstance(result['from_state'], str) or result['from_state'] is None)
        self.assertTrue(isinstance(result['to_state'], str) or result['to_state'] is None)
        self.assertTrue(isinstance(result['transition_description'], str) or result['transition_description'] is None)
        self.assertTrue(isinstance(result['action'], str) or result['action'] is None)

        print(f"{self.relationship_manager.relationships_matrix[idx_a][idx_b]=}")
        print(f"{self.relationship_manager.relationships_matrix[idx_b][idx_a]=}")

        # Third tick
        print("THIRD TICK")
        result, tick_id = self.relationship_manager.tick_get_relationship(5299113, 117227880, self.group_id, [5299113, 117227880, 17223924946])
        self.assertIsInstance(tick_id, int, "tick_id is not of type int")
        self.assertIsInstance(result, dict, "result is not of type dict")
        self.assertIsInstance(result['state_changed'], bool)
        self.assertTrue(isinstance(result['from_state'], str) or result['from_state'] is None)
        self.assertTrue(isinstance(result['to_state'], str) or result['to_state'] is None)
        self.assertTrue(isinstance(result['transition_description'], str) or result['transition_description'] is None)
        self.assertTrue(isinstance(result['action'], str) or result['action'] is None)

        print(f"{self.relationship_manager.relationships_matrix[idx_a][idx_b]=}")
        print(f"{self.relationship_manager.relationships_matrix[idx_b][idx_a]=}")

    def test_5_create_tick_snapshot(self):
        current_tick = self.relationship_manager.tick_counter[self.group_id-1]
        self.relationship_manager.create_tick_snapshot(self.group_id, [5299113, 117227880, 17223924946])
        print(f"{self.relationship_manager.history = }")
        self.assertEqual(self.relationship_manager.tick_counter[self.group_id-1], current_tick+1)

    def test_6_rewind_to_tick(self):

        tick_id = 0

        idx_a = self.relationship_manager.person_ids.index(117227880)
        idx_b = self.relationship_manager.person_ids.index(5299113)
        print(f"{self.relationship_manager.relationships_matrix[idx_a][idx_b]=}")
        print(f"{self.relationship_manager.relationships_matrix[idx_b][idx_a]=}")

        self.relationship_manager.rewind_to_tick(self.group_id, [5299113, 117227880, 17223924946], tick_id)
        self.assertEqual(self.relationship_manager.tick_counter[self.group_id-1], tick_id)

        print(f"{self.relationship_manager.relationships_matrix[idx_a][idx_b]=}")
        print(f"{self.relationship_manager.relationships_matrix[idx_b][idx_a]=}")

if __name__ == '__main__':
    unittest.main()
import unittest
from unittest.mock import patch, MagicMock
from llm_agent import FreeLLMAgent, VehicleConfig
import os
import json
from hf_token import HFTOKEN

class TestFreeLLMAgent(unittest.TestCase):
    def setUp(self):
        # Create test config
        self.config_path = "test_config.json"
        with open(self.config_path, 'w') as f:
            json.dump(VehicleConfig().model_dump(), f)
        
        # Mock environment variables
        os.environ["HF_TOKEN"] = HFTOKEN
    
    def tearDown(self):
        if os.path.exists(self.config_path):
            os.remove(self.config_path)
    
    @patch('requests.post')
    def test_huggingface_integration(self, mock_post):
        """Test Hugging Face API integration with debug output"""
        # Mock successful response
        mock_response = MagicMock()
        mock_response.json.return_value = [{
            'generated_text': json.dumps({
                "target_sequence": [{"x": 50, "y": -30, "depth": 20}],
                "electrical": {"main_light": 100},
                "transit_speed": 2.0
            })
        }]
        mock_response.raise_for_status.return_value = None
        mock_post.return_value = mock_response
        
        # Create agent with debug enabled
        agent = FreeLLMAgent(api_source="huggingface", debug=True)
        print("\n===== TESTING HUGGING FACE INTEGRATION =====")
        config = agent.generate_config(
            sensor_data={"depth": 5, "position": {"x": 0, "y": 0}},
            user_command="Go to (50,-30) at 20m depth"
        )
        
        self.assertEqual(config.target_sequence[0].x, 50)
        self.assertEqual(config.electrical.main_light, 100)
        print("===== TEST PASSED =====\n")
    
    @patch('requests.post', side_effect=Exception("API error"))
    def test_error_handling(self, mock_post):
        """Test fallback to default config on API failure"""
        # Create agent with debug enabled
        agent = FreeLLMAgent(debug=True)
        print("\n===== TESTING ERROR HANDLING =====")
        config = agent.generate_config(
            sensor_data={"error": "no connection"},
            user_command="Invalid command"
        )
        
        # Default config has no targets in sequence
        self.assertEqual(len(config.target_sequence), 0)
        print("===== TEST PASSED =====\n")

if __name__ == "__main__":
    print("\n" + "="*80)
    print("STARTING TEST SUITE")
    print("="*80 + "\n")
    unittest.main()
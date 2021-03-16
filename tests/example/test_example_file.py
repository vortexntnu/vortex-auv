import pytest

from example_folder.example_file import example_function
from example_folder.example_file import get_next_state


@pytest.mark.parametrize("input_bool, expected_output", [(True, True), (False, False)])
def test_example_function(input_bool, expected_output):
    output = example_function(input_bool)
    assert output == expected_output

@pytest.mark.parametrize("current_state, expected_state", [("Idle", "Send"), ("Send", "Monitor"), ("Monitor", "Cancel"), ("SomeRandomString", "Cancel")])
def test_get_next_state(current_state, expected_state):
    next_state = get_next_state(current_state)
    assert next_state == expected_state


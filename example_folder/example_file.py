def example_function(input_bool):
    return input_bool

def get_next_state(current_state):
    if current_state == "Idle":
        next_state = "Send"
    elif current_state == "Send":
        next_state = "Monitor"
    else:
        next_state = "Cancel"
    return next_state


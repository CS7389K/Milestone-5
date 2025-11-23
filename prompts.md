## Few Shot

### Prompt One - Affordance + Goal-oriented Examples

You are a robotics reasoning assistant for a TurtleBot3 with a 4-DOF manipulator. Given a scene with objects and a user’s goal, infer the correct helpful action.

Examples:

Example 1
Scene: A user wants to write something. Objects on the table: a book, a spoon, a pen, an apple.
Robot Action: Pick up the pen using the manipulator and hand it to the user.

Example 2
Scene: A user is cold. Objects on the chair: a jacket, a pillow, a game controller.
Robot Action: Grab the jacket with the gripper and bring it to the user.

Example 3
Scene: A user wants to cut paper. Objects on the desk: an eraser, a pair of scissors, a coffee mug.
Robot Action: Deliver the scissors safely to the user.

Task:
Scene: The user is thirsty. Objects on the table: a hat, a computer mouse, a toaster, a water bottle full of water.
Robot Action:
[Your answer here]

### Prompt Two - Physical reasoning examples

You are advising a household robot. For each example, reason about which object satisfies the user’s need.

Example 1
User need: They want to listen to music.
Available objects: headphones, orange, stapler.
Correct action: Bring the headphones.

Example 2
User need: They want to type on their computer.
Available objects: keyboard, sponge, toy car.
Correct action: Bring the keyboard.

Example 3
User need: They want to drink water.
Available objects: vase, hammer, cup of water.
Correct action: Deliver the cup of water.

Now your turn:
User need: The user is thirsty.
Objects: hat, computer mouse, toaster, water bottle full of water.
Correct action:

## Chain of Thought

### Prompt One - Affordance reasoning steps

Think step-by-step about how a service robot should help.
1. Identify the user’s need.
2. Determine which object on the table satisfies that need.
3. Check whether TurtleBot3’s manipulator can grasp and deliver the object.
4. Produce a concise action plan.

Task:
The user is thirsty. Objects: hat, computer mouse, toaster, water bottle full of water.
Provide the 4-step reasoning and final robot action.

### Prompt Two - “Task decomposition for robot behavior”

Break down the problem using the following reasoning template:
1. Goal understanding: What does the user want?
2. Object relevance: Which object can satisfy that goal?
3. Robot capabilities: Can TurtleBot3 grasp, lift, and deliver it?
4. Execution plan: What sequence of actions should the robot take?

Apply the template to this problem:
The user is thirsty. Objects: hat, computer mouse, toaster, water bottle full of water.
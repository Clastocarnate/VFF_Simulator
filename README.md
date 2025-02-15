# Virtual Force Field Simulator (VFF_Simulator)

This project demonstrates obstacle avoidance in ROS 2 using the Turtlesim package. It uses a Virtual Force Field (VFF) algorithm to control a bot turtle that navigates from the bottom left to the top right of the screen, avoiding five randomly spawned obstacle turtles. When the bot reaches the goal turtle, the simulation resets automatically.

## Demo Video

Watch the demo video below to see how to clone, build, and run the project:

[![VFF Simulator Demo]](https://youtu.be/b_KnmKoNRp0)



## Prerequisites

- **ROS 2** (Foxy, Galactic, or later)
- **turtlesim** package (usually included with ROS 2)
- **Python 3**

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Clastocarnate/VFF_Simulator.git
   cd VFF_Simulator
   ```
2. **Build the package:**
   ```bash
   colcon build --packages-select vff_turtle_bringup
   ```
3. **Source the setup file:**
   ```bash
   . install/setup.bash
   ```

## Usage

Launch the simulator using the following command:
```bash
ros2 launch vff_turtle_bringup vff_configure.py
```

This command starts the turtlesim node along with the VFF obstacle avoidance node. The simulation will:

- Spawn **5 obstacle turtles** at random positions in the middle of the canvas with:
  - At least a 1-unit (≈45 pixels) border from the canvas edges.
  - A minimum gap of 48 pixels (≈1.07 units) between each other, and from the bot and goal turtles.
- Spawn the **bot turtle** at the bottom left corner (position (1, 1)).
- Spawn the **goal turtle** at the top right corner (position (10, 10)).

The bot turtle uses the VFF algorithm to compute attractive and repulsive forces, guiding it toward the goal while avoiding collisions with the obstacles. Once it reaches the goal, the simulation resets and the process repeats.

## Algorithm Explanation

The obstacle avoidance algorithm is based on the Virtual Force Field (VFF) approach and consists of the following steps:

1. **Attractive Force:**
   - **Objective:** Pull the bot turtle toward the goal.
   - **Computation:** Calculate the difference between the goal’s and bot’s positions.
   - **Formula:**
     ```
     F_attr = k_attr * (goal_position - bot_position)
     ```
     where `k_attr` is a gain factor.

2. **Repulsive Force:**
   - **Objective:** Push the bot turtle away from obstacles to avoid collisions.
   - **Computation:** For each obstacle, if its distance `d` to the bot turtle is less than a threshold `d0` (e.g., 2.0 units), compute a repulsive force.
   - **Formula:**
     ```
     F_rep = k_rep * (1/d - 1/d0) / (d^2) * unit_vector
     ```
     where `k_rep` is the repulsion gain and the unit vector points from the obstacle to the bot.

3. **Net Force and Heading Adjustment:**
   - **Total Force:** Sum the attractive and repulsive forces:
     ```
     F_total = F_attr + F_rep
     ```
   - **Desired Heading:** Compute the direction using the arctangent of the force components:
     ```
     theta_desired = atan2(F_total_y, F_total_x)
     ```
   - **Control:** Use a proportional controller to adjust the bot’s angular velocity based on the error between `theta_desired` and its current orientation, while maintaining a constant forward speed.

4. **Obstacle Placement:**
   - **Constraints:**
     - Obstacle turtles are spawned within the region [1, 10] to maintain a 1-unit border from the canvas edges.
     - Each obstacle must be at least 48 pixels (≈1.07 units) away from every other turtle (obstacles, bot, and goal).
   - **Validation:** Candidate positions are generated randomly and validated against these constraints. If a candidate fails the gap check, a new position is generated (up to a maximum number of attempts).

5. **Reset Mechanism:**
   - When the bot turtle reaches within 0.5 units of the goal, the simulation:
     - Calls the `/reset` service of turtlesim.
     - Clears the stored pose information.
     - Respawns all turtles (obstacle, bot, and goal) for a fresh simulation run.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your improvements or bug fixes.


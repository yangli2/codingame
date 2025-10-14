import gymnasium as gym
from stable_baselines3 import PPO
import ppo_pod
import gymnasium_search_race.envs as gsr_envs

MAD_POD_ENV = "gymnasium_search_race:gymnasium_search_race/MadPodRacing-v2"

# Load the best pre-trained agent
# This requires stable-baselines3 to be installed
print("Loading pre-trained agent...")
agent = PPO.load("best_model.zip")
print("Agent loaded.")
print(agent.policy)

discrete_action_converter = ppo_pod.DiscreteActionConverter()

# Create the environment
# To display the graphics, set render_mode="human"
# The pre-trained agent works with the continuous action space environment.
env = gym.make(MAD_POD_ENV, render_mode="human")
policy = ppo_pod.Policy()

print("Starting simulation with the best agent for 3 episodes...")
for episode in range(3):
    # Reset the environment to get the initial state for the new episode
    obs, info = env.reset()
    done = False
    step_count = 0
    while not done:
        # Render the current state of the environment
        env.render()

        # Use the agent to predict the best action
        (
            rel_dx_ckpt_1,
            rel_dy_ckpt_1,
            sin_angle_ckpt_1,
            cos_angle_ckpt_1,
            rel_dx_ckpt_2,
            rel_dy_ckpt_2,
            sin_angle_ckpt_2,
            cos_angle_ckpt_2,
            rel_vy,
            rel_vx,
        ) = obs

        ckpt1_info = ppo_pod.CheckpointInfo(
            rel_dx=rel_dx_ckpt_1,
            rel_dy=rel_dy_ckpt_1,
            angle_sin=sin_angle_ckpt_1,
            angle_cos=cos_angle_ckpt_1,
        )
        ckpt2_info = ppo_pod.CheckpointInfo(
            rel_dx=rel_dx_ckpt_2,
            rel_dy=rel_dy_ckpt_2,
            angle_sin=sin_angle_ckpt_2,
            angle_cos=cos_angle_ckpt_2,
        )
        pod_info = ppo_pod.PodInfo(rel_vy=rel_vy, rel_vx=rel_vx)

        # discrete_action, _ = agent.predict(obs, deterministic=True)
        # action = discrete_action_converter.get_angle_and_thrust(discrete_action)
        action = policy.predict(ckpt1_info, ckpt2_info, pod_info)
        action = [action[0] / 18.0, action[1] / 200.0]
        print(action)

        # Execute the action in the environment
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        step_count += 1
    print(f"Episode {episode + 1} finished after {step_count} steps.")

# Close the environment
env.close()
print("Simulation finished.")

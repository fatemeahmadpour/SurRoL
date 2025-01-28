import gym
import surrol.gym
import time
import numpy as np

def test_environment():
    try:
        env = gym.make('SutureThreadManagement-v0', render_mode='human')
        print(f"Action space: {env.action_space}")
        print(f"Observation space: {env.observation_space}")
        
        for episode in range(3):
            print(f"\nEpisode {episode + 1}")
            obs = env.reset()
            episode_reward = 0
            
            for step in range(100):
                action = np.zeros(env.action_space.shape[0])
                
                if step < 30:
                    action[0:3] = np.random.uniform(-0.1, 0.1, 3)
                elif step < 60:
                    action[4] = 1.0
                    action[9] = 1.0
                else:
                    action[0:3] = np.random.uniform(-0.05, 0.05, 3)
                    action[5:8] = np.random.uniform(-0.05, 0.05, 3)
                
                obs, reward, done, info = env.step(action)
                episode_reward += reward
                
                time.sleep(0.01)
                
                if done:
                    print(f"Episode finished after {step} steps")
                    print(f"Thread positions: {obs['achieved_goal']}")
                    break
            
            print(f"Episode reward: {episode_reward:.2f}")
            if 'is_success' in info:
                print(f"Success: {info['is_success']}")
                
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if 'env' in locals():
            env.close()

if __name__ == "__main__":
    test_environment()
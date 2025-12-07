# Example 04: Physics Parameter Tuning

## Description
Demonstrates tuning physics engine parameters to balance simulation accuracy vs. real-time performance (RTF).

## Key Concepts
- Gravity modification (Earth, Moon, Mars, zero-G)
- Solver iteration tuning (iters: 20 vs 50 vs 500)
- Timestep adjustment (max_step_size)
- RTF measurement and optimization

## Experiments
1. **Gravity Variations**: Test humanoid behavior under different gravity (9.81, 3.71, 1.62, 0)
2. **Solver Tuning**: Measure RTF vs accuracy trade-off with different solver iterations
3. **Timestep Impact**: Compare 0.001s vs 0.0005s vs 0.002s timesteps

## Expected Insights
- Lower gravity → slower motion, higher jumps
- More solver iterations → better accuracy, lower RTF
- Smaller timestep → more stable, lower RTF

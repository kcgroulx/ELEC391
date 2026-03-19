#ifndef HAL_SIM_H
#define HAL_SIM_H

/**
 * @file hal_sim.h
 * @brief Extra declarations for the desktop HAL simulation.
 *
 * Include this alongside hal_interface.h in test files to access
 * simulation-only functions like hal_sim_printStats().
 */

/* Print a summary of motor travel, notes played, and simulated time.
 * Call at the end of your test main(). */
void hal_sim_printStats(void);

#endif /* HAL_SIM_H */

"""
FanEnv - Ambiente Gymnasium per il controllo in forza di un FAN tramite ESC.

La load cell misura:   F_lc = T - m*g*sin(theta) + bias + rumore
con T spinta del fan (dinamica del primo ordine del rotore, costante di tempo tau).

SAC osserva (normalizzato):
    [ F_lc / F_scale,
      errore / F_ref,
      dF_lc / F_scale,
      ultimo comando ESC (0..1),
      integrale errore / I_max ]

SAC decide:
    azione a in [-1, 1]  ->  ESC = (a + 1) / 2  in [0, 1]

Perturbazione (NON osservata):
    inclinazione della corsia, con cambi a gradino o a rampa.

Domain randomization (opzionale) per il trasferimento sim -> banco reale:
    guadagno fan, tau, massa, rumore e bias sensore, deadband ESC.
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces


class FanEnv(gym.Env):

    metadata = {"render_modes": []}

    def __init__(
        self,
        force_ref=3.0,              # [N]   forza desiderata sulla load cell
        dt=0.02,                    # [s]   periodo di campionamento (50 Hz)
        max_episode_steps=1000,     # [-]   durata episodio (20 s a 50 Hz)
        domain_randomization=True,
    ):
        super().__init__()

        # ==================================================
        # CONTROLLO
        # ==================================================
        self.force_ref = force_ref
        self.dt = dt
        self.max_episode_steps = max_episode_steps
        self.domain_randomization = domain_randomization

        # ==================================================
        # PARAMETRI FISICI NOMINALI
        # ==================================================
        self.g = 9.81
        self.mass_nom = 0.317       # [kg]  massa carrellino
        self.k_nom = 10.0           # [N]   spinta a ESC=100%:  T = k * ESC^2
        self.tau_nom = 0.15         # [s]   costante di tempo motore + elica
        self.deadband_nom = 0.01    # [-]   sotto questo comando il motore non gira
        self.noise_std_nom = 0.10   # [N]   rumore load cell
        self.max_angle = 45.0       # [deg] inclinazione massima corsia

        # ==================================================
        # NORMALIZZAZIONI / REWARD
        # ==================================================
        self.f_scale = 10.0         # [N]   scala per forza e delta forza
        self.i_max = 2.0            # [N*s] saturazione integrale (anti-windup)
        self.w_du = 5     #was 0.5        # peso penalità variazione comando
        self.w_u = 0.01             # peso (piccolo) sullo sforzo di controllo
        self.beta_df = 0.2          # coefficiente EMA per filtrare dF_lc (0=fermo, 1=nessun filtro)

        # ==================================================
        # SPAZI
        # ==================================================
        # azione simmetrica e normalizzata (consigliato per SAC)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )

        # osservazioni normalizzate, circa in [-1, 1] nel range operativo
        obs_high = np.array([10.0, 10.0, 10.0, 1.0, 1.0], dtype=np.float32)
        obs_low = np.array([-10.0, -10.0, -10.0, 0.0, -1.0], dtype=np.float32)
        self.observation_space = spaces.Box(
            low=obs_low, high=obs_high, dtype=np.float32
        )

        # stato interno (inizializzato in reset)
        self.thrust = 0.0
        self.angle = 0.0
        self.last_force = 0.0
        self.last_esc = 0.0
        self.integral = 0.0
        self.t = 0

    # ======================================================
    # UTILITY
    # ======================================================

    def _sample_parameters(self):
        """Parametri fisici dell'episodio (randomizzati o nominali)."""
        if self.domain_randomization:
            r = self.np_random
            self.mass = self.mass_nom * r.uniform(0.9, 1.1) # =SERVE
            self.k = self.k_nom * r.uniform(0.8, 1.2)
            self.tau = self.tau_nom * r.uniform(0.6, 1.6) #SERVE Simula il ritardo dinamico del sistema.
            self.deadband = r.uniform(0.01, 0.10)  # range allargato: include il nominale 0.01 usato in eval
            self.noise_std = r.uniform(0.05, 0.15)
            self.bias = r.uniform(-0.05, 0.05)
        else:
            self.mass = self.mass_nom
            self.k = self.k_nom
            self.tau = self.tau_nom
            self.deadband = self.deadband_nom
            self.noise_std = self.noise_std_nom
            self.bias = 0.0

        # coefficiente del filtro del primo ordine discretizzato (esatto)
        self.alpha = 1.0 - np.exp(-self.dt / self.tau)

    def _schedule_disturbance(self):
        """Programma il prossimo cambio di inclinazione (gradino o rampa)."""
        r = self.np_random
        self.angle_target = r.uniform(-self.max_angle, self.max_angle)
        self.steps_to_next_change = int(r.integers(100, 300))

        if r.random() < 0.5:
            self.angle_rate = np.inf                    # gradino
        else:
            ramp_time = r.uniform(0.5, 3.0)             # [s] rampa manuale
            self.angle_rate = abs(self.angle_target - self.angle) / ramp_time

    def _update_disturbance(self):
        """Evoluzione dell'inclinazione della piattaforma."""
        self.steps_to_next_change -= 1
        if self.steps_to_next_change <= 0:
            self._schedule_disturbance()

        diff = self.angle_target - self.angle
        max_step = self.angle_rate * self.dt
        self.angle += np.clip(diff, -max_step, max_step)

    def _esc_to_thrust_target(self, esc):
        """Spinta a regime per un dato comando ESC, con deadband."""
        if esc <= self.deadband:
            return 0.0
        u_eff = (esc - self.deadband) / (1.0 - self.deadband)
        return self.k * u_eff ** 2

    def _measure(self):
        """Lettura della load cell."""
        gravity_force = self.mass * self.g * np.sin(np.radians(self.angle))
        noise = self.np_random.normal(0.0, self.noise_std)
        return self.thrust - gravity_force + self.bias + noise

    def _get_obs(self, force, delta_force, error):
        obs = np.array(
            [
                force / self.f_scale,
                error / self.force_ref,
                delta_force / self.f_scale,
                self.last_esc,
                self.integral / self.i_max,
            ],
            dtype=np.float32,
        )
        return np.clip(obs, self.observation_space.low, self.observation_space.high)

    # ======================================================
    # RESET
    # ======================================================

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self._sample_parameters()

        self.thrust = 0.0
        self.last_esc = 0.0
        self.integral = 0.0
        self.df_filt = 0.0                  # stato del filtro EMA su dF_lc, azzerato a ogni episodio
        self.t = 0

        # inclinazione iniziale e primo cambio programmato
        self.angle = self.np_random.uniform(-self.max_angle, self.max_angle)
        self._schedule_disturbance()
        self.angle_target = self.angle      # il primo tratto parte già a regime

        # lettura reale a fan spento (include gravità, bias e rumore)
        force = self._measure()
        error = self.force_ref - force
        self.last_force = force

        obs = self._get_obs(force, 0.0, error)
        info = {"force": force, "force_ref": self.force_ref, "angle": self.angle, "ESC": 0.0}  # force_ref per coerenza con step
        return obs, info

    # ======================================================
    # STEP serve a simulare un passo di tempo dell'ambiente, aggiornando lo stato interno in base all'azione fornita e restituendo l'osservazione successiva, la ricompensa, e altre informazioni rilevanti.
    # ======================================================

    def step(self, action):
        self.t += 1

        # azione [-1, 1] -> ESC [0, 1]
        a = float(np.clip(action[0], -1.0, 1.0))
        esc = 0.5 * (a + 1.0)

        # disturbo
        self._update_disturbance()

        # dinamica rotore (primo ordine, discretizzazione esatta)
        thrust_target = self._esc_to_thrust_target(esc)
        self.thrust += self.alpha * (thrust_target - self.thrust)

        # misura
        force = self._measure()
        # derivata della forza filtrata con EMA: la differenza grezza è dominata dal rumore
        self.df_filt += self.beta_df * ((force - self.last_force) - self.df_filt)
        delta_force = self.df_filt
        error = self.force_ref - force

        # integrale errore con anti-windup (saturazione)
        self.integral = float(
            np.clip(self.integral + error * self.dt, -self.i_max, self.i_max)
        )

        # reward normalizzato
        e_n = error / self.force_ref
        du = esc - self.last_esc
        reward = -(e_n ** 2) - self.w_du * du ** 2 - self.w_u * esc ** 2

        # aggiornamento memoria
        self.last_force = force
        self.last_esc = esc

        obs = self._get_obs(force, delta_force, error)

        terminated = False
        truncated = self.t >= self.max_episode_steps

        info = {
            "force": force,
            "force_ref": self.force_ref,  # riferimento dello stesso passo (serve a evaluate)
            "angle": self.angle,
            "ESC": esc,
            "thrust": self.thrust,
        }

        return obs, float(reward), terminated, truncated, info


# ==========================================================
# TEST RAPIDO + ESEMPIO DI TRAINING
# ==========================================================

if __name__ == "__main__":
    import numpy as np
    from gymnasium.utils.env_checker import check_env

    env = FanEnv()
    check_env(env)
    print("check_env: OK")

    # rollout random riproducibile
    obs, info = env.reset(seed=0)
    env.action_space.seed(0)
    total = 0.0
    for _ in range(env.max_episode_steps):
        obs, r, term, trunc, info = env.step(env.action_space.sample())
        total += r
        if term or trunc:
            break
    print(f"Reward episodio random: {total:.1f}")

    try:
        from stable_baselines3 import SAC
        from stable_baselines3.common.monitor import Monitor
        from stable_baselines3.common.callbacks import EvalCallback
    except ImportError:
        print("stable-baselines3 non installato: training saltato.")
        raise SystemExit

    train_env = Monitor(FanEnv()) #traini 
    eval_cb_env = Monitor(FanEnv(domain_randomization=False)) #testi per dare un voto

    eval_cb = EvalCallback(
        eval_cb_env,
        best_model_save_path="./best_sac_fan",
        log_path="./logs",
        eval_freq=5_000,
        n_eval_episodes=5,
        deterministic=True,
    )

    model = SAC(
        "MlpPolicy",
        train_env,
        learning_rate=3e-4,
        buffer_size=200_000,
        batch_size=256,
        gamma=0.98,
        verbose=1,
        seed=0,
    )
    model.learn(total_timesteps=100_000, callback=eval_cb)
    model.save("sac_fan")

    # per la valutazione usa il miglior modello salvato da EvalCallback, non l'ultimo
    model = SAC.load("./best_sac_fan/best_model")

    # valutazione: nominale e randomizzata, più seed
     # valutazione: nominale e randomizzata, più seed
    def evaluate(env, n_ep=10):
        rms, du = [], []
        for ep in range(n_ep):
            obs, _ = env.reset(seed=100 + ep)
            errs, acts = [], []
            for _ in range(env.max_episode_steps):
                a, _ = model.predict(obs, deterministic=True)
                obs, r, term, trunc, info = env.step(a)
                errs.append(info["force_ref"] - info["force"])
                acts.append(np.asarray(a).ravel())
                if term or trunc:
                    break
            errs, acts = np.array(errs), np.array(acts)
            rms.append(np.sqrt(np.mean(errs**2)))
            du.append(np.mean(np.abs(np.diff(acts, axis=0))))
        return np.mean(rms), np.std(rms), np.mean(du)

    for name, dr in [("nominale", False), ("randomizzato", True)]:
        m, s, d = evaluate(FanEnv(domain_randomization=dr))
        print(f"[{name}] RMS errore: {m:.3f} ± {s:.3f} N | mean|Δu|: {d:.4f}")

    # curva di apprendimento da EvalCallback
    import matplotlib.pyplot as plt

    data = np.load("./logs/evaluations.npz")
    timesteps = data["timesteps"]          # (n_eval,)
    results = data["results"]              # (n_eval, n_eval_episodes)
    mean_r = results.mean(axis=1)
    std_r = results.std(axis=1)

    plt.figure(figsize=(8, 5))
    plt.plot(timesteps, mean_r, marker="o", label="Reward medio (5 ep)")
    plt.fill_between(timesteps, mean_r - std_r, mean_r + std_r, alpha=0.25, label="±1σ")
    plt.title("Curva di apprendimento – EvalCallback")
    plt.xlabel("Timesteps")
    plt.ylabel("Reward episodio")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("learning_curve.png", dpi=150)
    plt.show()
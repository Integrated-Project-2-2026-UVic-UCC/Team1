
# 🐕 Custom Quadruped RL Controller (MJLab)

Proyecto de simulación y entrenamiento por Aprendizaje por Refuerzo (Reinforcement Learning) para un robot cuadrúpedo custom impreso en 3D. El entorno está construido sobre **MuJoCo** utilizando el framework acelerado por GPU **mjlab**.

### 🏃‍♂️ Locomotion Demo
![Demo del robot caminando](demos/Screencast from 16-03-26 10_20_32.gif)

---

## ⚠️ Pruebas Experimentales de Recompensas (Reward Shaping)
Actualmente, el proyecto se encuentra en una fase de **experimentación de recompensas**. 

Estamos iterando sobre los pesos (*weights*) del entorno para evitar "mínimos locales" (como que el robot se quede quieto para maximizar el premio por estar de pie) y forzar la exploración hacia adelante. La configuración actual penaliza la inestabilidad del torso pero ofrece grandes incentivos por alcanzar la velocidad lineal deseada (1.0 m/s), suprimiendo temporalmente las penalizaciones de *foot clearance* para facilitar las primeras fases del *gait* (trote).

## ⚙️ Especificaciones del Modelo Físico (Sim-to-Real)
El archivo `quadruped_robot.xml` y sus constantes están rigurosamente modelados a partir del hardware real para garantizar una transferencia limpia a la realidad:

*   **Masa Total:** 2.5 kg.
*   **Cinemática:** 12 DoF. Motores concentrados en la cadera (Hip) con transmisión por varilla para la rodilla (KFE), minimizando la inercia de las piernas.
*   **Actuadores (Servos de 270°):** Modelados como motores DC con controlador PD interno.
    *   **Stall Torque:** ~2.45 N·m (25 kg·cm).
    *   **Pérdida Dinámica:** Implementada mediante un factor de *Damping* experimental para simular la pérdida de torque a altas velocidades bajo carga.
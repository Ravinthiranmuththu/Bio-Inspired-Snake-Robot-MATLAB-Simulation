# Mathematical model and engineering limitations

## Degrees of freedom
12 rigid links are joined by 11 actuated revolute joints. Odd-indexed joints rotate around their **local Z axes** (yaw); even-indexed joints rotate around their **local Y axes** (pitch). Forward kinematics accumulates rotations and link translations in 3D.

## Prescribed joint motion versus emergent body translation
Joint **angles** are prescribed using a sinusoidal travelling wave, but planar rigid-body velocity is solved at each instant from a quasi-static resistive-force balance. This is not a torque-controlled articulated multibody dynamics model.

For segment j, the planar velocity is `v_j = V + omega [-r_y; r_x] + v_shape,j`. The drag tensor is `D_j = c_parallel t_j t_j' + c_normal n_j n_j'`. Force and yaw moment balance produce `M [Vx; Vy; omega] = -b`. A regularized yaw drag term is included. The pose uses explicit Euler integration.

## Limitations that must accompany any demo
1. Pitch joints influence 3D rendering but **not** planar friction-force calculations.
2. The 3D robot is not constrained against penetrating the ground or self-intersecting.
3. No rigid-body inertia, mass distribution, gravity, joint torques, actuator dynamics, Coulomb/stick-slip friction, contact detection, or collision handling.
4. Drag coefficients are illustrative and not calibrated to physical hardware.
5. The first-link origin is used as the body pose reference; output distance is not a validated center-of-mass trajectory.
6. Numerical accuracy and convergence have not been established; time-step sensitivity should be tested.
7. No obstacle avoidance, waypoint tracking, sidewinding or concertina gait implementation is included.

## Suggested verification tests
- Set gait amplitudes to zero: the body should remain stationary.
- Set `cParallel = cNormal`: investigate whether net displacement decreases toward zero under isotropic resistance.
- Reduce `dt` by increasing `fps` and compare final pose (time-step convergence).
- Reverse the travelling-wave phase and inspect displacement direction.
- Compare zero pitch amplitude to nonzero pitch amplitude: current planar solver is unchanged, demonstrating the model limitation.
- Check the planar 3x3 system conditioning and residual force/moment.

## Next modelling milestone
Develop a contact-consistent 3D multibody model (Simscape Multibody or another validated dynamics engine), define mass/inertia for each link, actuated revolute joints, collision geometry, normal contact and directional friction, and compare its displacement to the simplified model.

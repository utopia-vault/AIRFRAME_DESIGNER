# V4 with four front motors side by side: weight and front-motor angles

Source: Fusion "SMALL_SCALE_V4.2 - Four front motors - Side by side" v1, the "Side ESCs" variant, read over
the Fusion MCP (127.0.0.1:27182) on 2026-10-23: `fusion_four_front_side_by_side_bodies.json`.
Design: `airframes/atlas_v4_front4_side_by_side.json`, built by `front4.py` from `atlas_v4_fusion_v8_h18_p_0`.

## Weight
| | V4 (3 front) | V4 front4 |
|---|---|---|
| mass | 13.40 kg | **14.63 kg** (+0.70 battery, +0.33 EDF, +0.20 ESC) |
| CG (FRD, m) | -0.1994, 0.0012, -0.4319 | -0.1745, 0.0012, -0.4407 |
| Ixx Iyy Izz | 0.886 1.193 1.637 | 0.918 1.397 1.841 |
| natural hover pitch | 18.1 deg | 17.5 deg |

From the CAD: M7/M8 are unchanged, M9 became two vertical motors at y -/+0.052 (1 cm higher), the centre
front ESC moved beside them and a second was added. Assumed: the fourth 700 g pack is not in the CAD; four
packs 55 mm apart, centred where the three are. Not counted: the new cradle against the old one, extra bolts.

## Authority (static: PX4's pseudo-inverse allocation, room around the hover trim, through the inertia)
Angles in the app's convention: tilt forward from vertical, cant outward (negative = leaning inward).
M7/M8 = outer pair at x 0.255, M9/M10 = side-by-side pair at x 0.355. Hover pitch re-solved per design.

| design | M7/M8 cant, tilt | M9/M10 cant, tilt | hover pitch | roll | pitch | yaw (rad/s^2) | trim util |
|---|---|---|---|---|---|---|---|
| V4, 3 front (today) | -30, 0 | (M9 0, 0) | 18.1 | 7.67 | 20.27 | 3.01 | 0.36-0.66 |
| front4 as drawn | -30, 0 | 0, 0 | 17.5 | 6.88 | 19.32 | 2.42 | 0.30-0.71 |
| A free optimum | -43.4, 20.0 | +58.3, -1.7 | 22.6 | 12.16 | 10.37 | 10.37 | 0.48-0.72 |
| B pitch >= 17 | +30.0, -15.1 | -51.4, 25.9 | 19.2 | 11.41 | 17.00 | 9.95 | 0.48-0.66 |
| **C buildable** | **-45.0, 22.6** | **+30.0, 11.1** | **23.8** | **11.96** | **18.55** | **8.84** | 0.42-0.64 |

C = pitch >= 17 rad/s^2, every angle within 45 deg, M9/M10 cant within 30 deg. As built (1000 draws:
+/-2 deg mounting error per rotor, CG +/-10 mm, thrust +/-5 %), C keeps roll 10.4 / pitch 16.4 / yaw 7.8 at
the 5th percentile; every draw hovers. Files: `airframe_front4_C_buildable.json`, `mc_front_tilt.*`,
`mc_variants.*`.

Not covered: no SITL flight (SITL parameter seeding is failing on this machine today: the repo's own
`test_hover_scenario_on_quad_x` fails the same way, "seeded PX4 parameters not in effect"). Duct clearance
and exhaust paths at the new angles are not checked. Canted-thrust yaw comes with about 3.3 N of side force
per N m. C's 23.8 deg hover pitch changes SENS_BOARD_Y_OFF, the H-flow beam angle and the park geometry.

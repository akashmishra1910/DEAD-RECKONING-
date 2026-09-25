import os
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

def draw_card(ax, x, y, w, h, stage_num, title, items, header_bg, border_color, card_bg='#ffffff'):
    # Outer card
    card = FancyBboxPatch((x, y), w, h,
                          boxstyle="round,pad=0.012,rounding_size=0.025",
                          facecolor=card_bg, edgecolor=border_color,
                          linewidth=1.5, zorder=3)
    ax.add_patch(card)
    
    # Header bar
    th = 5.0
    header = FancyBboxPatch((x, y + h - th), w, th,
                            boxstyle="round,pad=0.012,rounding_size=0.025",
                            facecolor=header_bg, edgecolor=border_color,
                            linewidth=1.0, zorder=4)
    ax.add_patch(header)
    
    # Header text
    full_title = f"{stage_num}. {title}" if stage_num else title
    ax.text(x + w/2, y + h - th/2, full_title,
            ha='center', va='center', fontsize=8.8, fontweight='bold',
            color='white', zorder=5)
    
    # Body container: render structured rows
    body_h = h - th
    n_items = len(items)
    spacing = (body_h - 1.5) / max(1, n_items)
    
    for i, item in enumerate(items):
        y_center = y + body_h - 1.0 - (i + 0.5) * spacing
        head = item.get('head', '')
        lines = item.get('lines', [])
        highlight_idx = item.get('highlight_idx', -1)
        
        # Sub-heading
        ax.text(x + 1.0, y_center + 1.8, head,
                ha='left', va='center', fontsize=7.6, fontweight='bold',
                color='#0f172a', zorder=5)
        
        # Supporting description lines
        for j, line in enumerate(lines):
            is_hl = (j == highlight_idx)
            fontweight = 'bold' if is_hl else 'normal'
            color = ('#7c3aed' if '1.72' in line else '#0284c7' if '0.88' in line else '#16a34a' if '%' in line else '#1e293b') if is_hl else '#475569'
            ax.text(x + 1.0, y_center + 0.3 - j * 1.5, line,
                    ha='left', va='center', fontsize=6.8, fontweight=fontweight,
                    color=color, zorder=5)

def draw_aux_card(ax, x, y, w, h, title, items, header_bg, border_color, card_bg='#ffffff'):
    card = FancyBboxPatch((x, y), w, h,
                          boxstyle="round,pad=0.012,rounding_size=0.025",
                          facecolor=card_bg, edgecolor=border_color,
                          linewidth=1.4, zorder=3)
    ax.add_patch(card)
    
    th = 4.4
    header = FancyBboxPatch((x, y + h - th), w, th,
                            boxstyle="round,pad=0.012,rounding_size=0.025",
                            facecolor=header_bg, edgecolor=border_color,
                            linewidth=1.0, zorder=4)
    ax.add_patch(header)
    
    ax.text(x + w/2, y + h - th/2, title,
            ha='center', va='center', fontsize=8.2, fontweight='bold',
            color='white', zorder=5)
    
    body_h = h - th
    n_items = len(items)
    spacing = (body_h - 1.5) / max(1, n_items)
    for i, itm in enumerate(items):
        y_pos = y + body_h - 1.0 - (i + 0.5) * spacing
        ax.text(x + 1.2, y_pos + 1.2, itm['title'],
                ha='left', va='center', fontsize=7.4, fontweight='bold',
                color='#1e293b', zorder=5)
        for k, l in enumerate(itm['lines']):
            is_hl = itm.get('hl', False)
            ax.text(x + 1.2, y_pos - 0.4 - k * 1.5, l,
                    ha='left', va='center', fontsize=6.7,
                    fontweight='bold' if is_hl else 'normal',
                    color='#0284c7' if is_hl else '#475569', zorder=5)

def draw_arrow_badge(ax, x, y, text, color='#1e293b', bg='white', border='#94a3b8', fontsize=6.2):
    ax.text(x, y, text, ha='center', va='center',
            fontsize=fontsize, fontweight='bold', color=color,
            bbox=dict(boxstyle='round,pad=0.10', facecolor=bg, edgecolor=border, lw=0.6, alpha=0.98),
            zorder=8)

def main():
    fig = plt.figure(figsize=(18.0, 9.8), dpi=300)
    ax = fig.add_subplot(111)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis('off')
    
    fig.patch.set_facecolor('#ffffff')
    ax.set_facecolor('#ffffff')
    
    # Title Block
    ax.text(50, 97.6, 'INTELLIGENT DEAD RECKONING (IDR) & 3D MAP-MATCHING ARCHITECTURE',
            ha='center', va='center', fontsize=14.0, fontweight='bold', color='#0f172a')
    ax.text(50, 95.1, 'End-to-End Navigation Dataflow: Sensor Ingestion, Dynamic Calibration, AI Speed Engine, 15-State ESKF Core, and 3D Road Snapping',
            ha='center', va='center', fontsize=8.6, fontstyle='italic', color='#64748b')

    # =============================================================
    # 5 HORIZONTAL MAINLINE STAGES (Y: 47.0 to 86.0)
    # Width = 14.2 each. 4 Gaps = 5.75 each. Left margin = 3.5
    # =============================================================
    w = 14.2
    gap = 5.75
    y_m = 47.0
    h_m = 39.0
    
    x1 = 3.5
    x2 = x1 + w + gap   # 23.45
    x3 = x2 + w + gap   # 43.40
    x4 = x3 + w + gap   # 63.35
    x5 = x4 + w + gap   # 83.30

    # -------------------------------------------------------------
    # STAGE 1: SENSOR INGESTION
    # -------------------------------------------------------------
    draw_card(ax, x1, y_m, w, h_m, "1", "SENSOR INGESTION",
              [
                  {
                      "head": "• Smartphone IMU (100 Hz)",
                      "lines": ["Tri-axial accel f_b & gyro omega_b", "Uncalibrated device body frame {b}", "Subject to thermal drift and bias"]
                  },
                  {
                      "head": "• Barometer (10 Hz)",
                      "lines": ["Air pressure hypsometric altitude", "Vertical velocity constraint (v_z)"]
                  },
                  {
                      "head": "• GNSS Receiver (1 Hz)",
                      "lines": ["PVT state fixes & HDOP metrics", "Subject to urban canyon outages"]
                  }
              ],
              "#334155", "#334155", "#f8fafc")

    # -------------------------------------------------------------
    # STAGE 2: DYNAMIC CALIBRATION
    # -------------------------------------------------------------
    draw_card(ax, x2, y_m, w, h_m, "2", "DYNAMIC CALIBRATION",
              [
                  {
                      "head": "• Gravity Leveling",
                      "lines": ["Static roll phi_m & pitch theta_m", "Decomposes gravity vector"]
                  },
                  {
                      "head": "• Surge Dynamic Yaw",
                      "lines": ["Forward acceleration tracking", "Rotation matrix R_b^v ({b} -> {v})"]
                  },
                  {
                      "head": "• Road Anomaly Damping",
                      "lines": ["Dual-tier jerk filter (|a_z| > 3g)", "Rejects potholes & chassis shocks"]
                  }
              ],
              "#d97706", "#d97706", "#fffbeb")

    # -------------------------------------------------------------
    # STAGE 3: AI PSEUDO-VELOCITY
    # -------------------------------------------------------------
    draw_card(ax, x3, y_m, w, h_m, "3", "AI PSEUDO-VELOCITY",
              [
                  {
                      "head": "• Rolling Feature Extractor",
                      "lines": ["1.0 s sliding window (100 Hz)", "Surge energy, variance & jerk"]
                  },
                  {
                      "head": "• GBDT Speed Regressor",
                      "lines": ["LightGBM forward speed v_ai", "Replaces wheel odometry", "RMSE: 1.72 m/s | < 0.20 ms"],
                      "highlight_idx": 2
                  },
                  {
                      "head": "• Virtual Sensor Synthesizer",
                      "lines": ["z_ai = [v_ai, 0, 0]^T vector", "Dynamic variance R_ai = f(feat)"]
                  }
              ],
              "#8e44ad", "#8e44ad", "#faf5ff")

    # -------------------------------------------------------------
    # STAGE 4: 15-STATE ESKF CORE
    # -------------------------------------------------------------
    draw_card(ax, x4, y_m, w, h_m, "4", "15-STATE ESKF CORE",
              [
                  {
                      "head": "• Nominal Mechanization",
                      "lines": ["100 Hz p, v, q state integration", "Propagates covariance P_{k|k-1}"]
                  },
                  {
                      "head": "• Measurement Multiplexer",
                      "lines": ["Mode A: GNSS fixes (clear sky)", "Mode B: AI speed + NHC (outage)", "Fused 3D Vel RMSE: 0.88 m/s"],
                      "highlight_idx": 2
                  },
                  {
                      "head": "• Error Injection & Reset",
                      "lines": ["Joseph-form covariance reset", "Closed-loop bias tracking b_a, b_g"]
                  }
              ],
              "#0284c7", "#0284c7", "#f0f9ff")

    # -------------------------------------------------------------
    # STAGE 5: 3D MAP-MATCHING & OUT
    # -------------------------------------------------------------
    draw_card(ax, x5, y_m, w, h_m, "5", "3D MAP-MATCHING & OUT",
              [
                  {
                      "head": "• Topological Snapping",
                      "lines": ["Multi-hypothesis HMM matcher", "Projects trajectory to centerlines"]
                  },
                  {
                      "head": "• Orthogonal Constraint",
                      "lines": ["Eliminates cross-track drift", "Aligns elevations to road grades"]
                  },
                  {
                      "head": "• High-Integrity Output",
                      "lines": ["Continuous 100 Hz 6-DOF solution", "Immune to satellite multipath", "Outage Drift: 1.72% – 2.80%"],
                      "highlight_idx": 2
                  }
              ],
              "#16a34a", "#16a34a", "#f0fdf4")

    # =============================================================
    # MAINLINE HORIZONTAL ARROWS & GAP BADGES (Y: 66.5)
    # =============================================================
    ay = 66.5
    
    # 1 -> 2
    ax.annotate('', xy=(x2, ay), xytext=(x1 + w, ay),
                arrowprops=dict(arrowstyle='->,head_width=0.32,head_length=0.44', lw=2.0, color='#334155'), zorder=6)
    draw_arrow_badge(ax, (x1 + w + x2)/2, ay, "Raw IMU", color='#334155')

    # 2 -> 3
    ax.annotate('', xy=(x3, ay), xytext=(x2 + w, ay),
                arrowprops=dict(arrowstyle='->,head_width=0.32,head_length=0.44', lw=2.0, color='#d97706'), zorder=6)
    draw_arrow_badge(ax, (x2 + w + x3)/2, ay, "R_b^v Align", color='#d97706')

    # 3 -> 4
    ax.annotate('', xy=(x4, ay), xytext=(x3 + w, ay),
                arrowprops=dict(arrowstyle='->,head_width=0.32,head_length=0.44', lw=2.2, color='#8e44ad'), zorder=6)
    draw_arrow_badge(ax, (x3 + w + x4)/2, ay, "v_ai (NHC)", color='#8e44ad')

    # 4 -> 5
    ax.annotate('', xy=(x5, ay), xytext=(x4 + w, ay),
                arrowprops=dict(arrowstyle='->,head_width=0.32,head_length=0.44', lw=2.2, color='#0284c7'), zorder=6)
    draw_arrow_badge(ax, (x4 + w + x5)/2, ay, "x_eskf", color='#0284c7')

    # Final Output arrow on right
    ax.annotate('', xy=(99.5, ay), xytext=(x5 + w, ay),
                arrowprops=dict(arrowstyle='->,head_width=0.34,head_length=0.46', lw=2.4, color='#16a34a'), zorder=6)

    # =============================================================
    # CLOSED-LOOP FEEDBACK LOOP (TOP CORRIDOR: Y: 86.0 to 91.0)
    # =============================================================
    ax.plot([x4 + w/2, x4 + w/2, x2 + w/2, x2 + w/2], [86.0, 91.0, 91.0, 86.0],
            color='#0284c7', lw=1.6, linestyle='--', zorder=6)
    ax.annotate('', xy=(x2 + w/2, 86.0), xytext=(x2 + w/2, 87.5),
                arrowprops=dict(arrowstyle='->,head_width=0.30,head_length=0.40', lw=1.6, color='#0284c7'), zorder=6)
    draw_arrow_badge(ax, (x2 + x4 + w)/2, 91.0, "Closed-Loop Bias Reset (delta_b_a, delta_b_g, delta_theta)",
                     color='#0284c7', bg='#f0f9ff', border='#0284c7', fontsize=7.2)

    # =============================================================
    # BOTTOM SUPPORTING LAYER (Y: 10.0 to 33.0, height = 23.0)
    # Symmetrical 4-block layout across entire bottom width!
    # =============================================================
    y_b = 10.0
    h_b = 23.0
    
    # BOX 1 (under Stage 1): KEY SYSTEM METRICS
    draw_aux_card(ax, x1, y_b, w, h_b,
                  "KEY BENCHMARK METRICS",
                  [
                      {
                          "title": "• Processing Rate:",
                          "lines": ["100 Hz Continuous Mechanization", "< 0.20 ms GBDT Tree Latency"],
                          "hl": False
                      },
                      {
                          "title": "• Fused Accuracy:",
                          "lines": ["0.88 m/s Fused 3D Velocity RMSE", "1.72% – 2.80% Extended Drift (<10%)"],
                          "hl": True
                      }
                  ],
                  "#1e293b", "#1e293b", "#f8fafc")

    # BOX 2 (under Stage 2 & 3): KINEMATIC CONSTRAINTS ENGINE
    draw_aux_card(ax, x2, y_b, (x3 + w) - x2, h_b,
                  "KINEMATIC CONSTRAINTS ENGINE",
                  [
                      {
                          "title": "• Zero-Velocity Update (ZUPT):",
                          "lines": ["Multi-axis accel variance gating detects vehicle standstill.", "Clamps velocity to zero (v = 0), completely halting static drift accumulation."]
                      },
                      {
                          "title": "• Non-Holonomic Constraints (NHC):",
                          "lines": ["Exploits land-vehicle dynamics: wheels cannot slip sideways (v_y^v ~ 0)", "or bounce off pavement (v_z^v ~ 0), bounding lateral and vertical error growth."]
                      }
                  ],
                  "#e11d48", "#e11d48", "#fff1f2")

    # BOX 3 (under Stage 4): ABSOLUTE OBSERVATIONS
    draw_aux_card(ax, x4, y_b, w, h_b,
                  "ABSOLUTE OBSERVATIONS",
                  [
                      {
                          "title": "• GNSS Mode A (1 Hz):",
                          "lines": ["Absolute PVT position/speed updates", "Active during healthy satellite line-of-sight."]
                      },
                      {
                          "title": "• Barometer (10 Hz):",
                          "lines": ["Hypsometric altitude constraint", "Aids vertical velocity & overpasses."]
                      }
                  ],
                  "#475569", "#475569", "#f1f5f9")

    # BOX 4 (under Stage 5): 3D ROAD NETWORK GRAPH
    draw_aux_card(ax, x5, y_b, w, h_b,
                  "3D ROAD NETWORK GRAPH",
                  [
                      {
                          "title": "• HD Centerlines:",
                          "lines": ["Metric roadway centerline splines", "Provides geometric road anchor."]
                      },
                      {
                          "title": "• 3D Grade Heights:",
                          "lines": ["Resolves multi-level flyovers and", "overpass vertical stack ambiguities."]
                      }
                  ],
                  "#15803d", "#15803d", "#f0fdf4")

    # =============================================================
    # VERTICAL INJECTION ARROWS (BOTTOM TO TOP)
    # =============================================================
    # Aux 2 -> Stage 3 (AI Speed Engine)
    ax.annotate('', xy=(x3 + w/2, y_m), xytext=(x3 + w/2, y_b + h_b),
                arrowprops=dict(arrowstyle='->,head_width=0.32,head_length=0.42', lw=1.8, color='#e11d48'), zorder=6)
    draw_arrow_badge(ax, x3 + w/2, (y_b + h_b + y_m)/2, "NHC & ZUPT Gating", color='#e11d48', bg='#fff1f2', border='#e11d48')

    # Aux 3 -> Stage 4 (ESKF Core)
    ax.annotate('', xy=(x4 + w/2, y_m), xytext=(x4 + w/2, y_b + h_b),
                arrowprops=dict(arrowstyle='->,head_width=0.32,head_length=0.42', lw=1.8, color='#475569'), zorder=6)
    draw_arrow_badge(ax, x4 + w/2, (y_b + h_b + y_m)/2, "Mode A: GNSS / Baro", color='#475569', bg='#f1f5f9', border='#475569')

    # Aux 4 -> Stage 5 (Map-Matching)
    ax.annotate('', xy=(x5 + w/2, y_m), xytext=(x5 + w/2, y_b + h_b),
                arrowprops=dict(arrowstyle='->,head_width=0.32,head_length=0.42', lw=1.8, color='#15803d'), zorder=6)
    draw_arrow_badge(ax, x5 + w/2, (y_b + h_b + y_m)/2, "Road Centerlines", color='#15803d', bg='#f0fdf4', border='#15803d')

    # Footer note
    footer_text = "Mainline 100 Hz Kinematic Mechanization   —   Closed-Loop Bias Reset   —   Kinematic & Map Constraints"
    ax.text(50, 4.0, footer_text, ha='center', va='center', fontsize=8.2, fontweight='semibold', color='#64748b')

    out_path = "plot_process_flow.png"
    fig.savefig(out_path, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none', bbox_inches='tight')
    print(f"Saved masterpiece process flow diagram to: {out_path}")

if __name__ == "__main__":
    main()

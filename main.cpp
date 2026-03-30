
#include "draw.hpp"
#include "geometry.hpp"
#include "simulation.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
using namespace std;

// Global Variables
bool initialized = false;
bool all_done = false;
vector<pair<double, double>> waypoints;
int current_idx = 0;

double last_px = 0, last_py = 0;
int stuck_count = 0;
int escape_mode = 0;
int blocked_cd = 0;
int wp_timer = 0;
int last_wp = -1;

double prev_steer = 0;
double prev_a = 0;
double prev_danger = 0;

vector<double> ema_scan;

double pdist(pair<double, double> a, pair<double, double> b) {
  return sqrt((a.first - b.first) * (a.first - b.first) +
              (a.second - b.second) * (a.second - b.second));
}

int main() {
  agent myagent;

  myagent.calculate_1 =
      [&](const envmap &curmap,
          const array<pair<point, point>, playercount> &playerdata,
          const array<point, rays> &raycasts, const agent &curplayer, ftype &a,
          ftype &steer) {
        double px = playerdata[0].first.x;
        double py = playerdata[0].first.y;
        double yaw = playerdata[0].second.y;
        double spd = playerdata[0].second.x;
        pair<double, double> pos = {px, py};

        vector<double> scan(rays);
        for (int i = 0; i < rays; i++) {
          double dx = raycasts[i].x - px;
          double dy = raycasts[i].y - py;
          scan[i] = sqrt(dx * dx + dy * dy);
        }
        double min_dist = *min_element(scan.begin(), scan.end());

        // Waypoint
        if (!initialized) {
          int grid = 8;
          vector<pair<double, double>> freePoints;
          for (int i = 0; i < grid; i++)
            for (int j = 0; j < grid; j++) {
              double wx = -0.8 + 1.6 * i / (grid - 1);
              double wy = -0.8 + 1.6 * j / (grid - 1);
              bool blocked = false;
              for (int k = 0; k < (int)curmap.size() - 1; k++)
                if (contains(curmap[k], point(wx, wy))) {
                  blocked = true;
                  break;
                }
              if (!blocked)
                freePoints.push_back({wx, wy});
            }

          vector<pair<double, double>> centroids;
          for (int i = 0; i < (int)freePoints.size(); i++) {
            int n1 = -1, n2 = -1;
            double d1 = 1e9, d2 = 1e9;
            for (int j = 0; j < (int)freePoints.size(); j++) {
              if (i == j)
                continue;
              double d = pdist(freePoints[i], freePoints[j]);
              if (d < d1) {
                d2 = d1;
                n2 = n1;
                d1 = d;
                n1 = j;
              } else if (d < d2) {
                d2 = d;
                n2 = j;
              }
            }
            if (n1 == -1 || n2 == -1)
              continue;
            centroids.push_back({(freePoints[i].first + freePoints[n1].first +
                                  freePoints[n2].first) /
                                     3.0,
                                 (freePoints[i].second + freePoints[n1].second +
                                  freePoints[n2].second) /
                                     3.0});
          }

          int step = max(1, (int)centroids.size() / 30);
          for (int i = 0;
               i < (int)centroids.size() && (int)waypoints.size() < 30;
               i += step)
            waypoints.push_back(centroids[i]);

          pair<double, double> cur = {px, py};
          vector<pair<double, double>> sorted;
          vector<bool> used(waypoints.size(), false);
          for (int i = 0; i < (int)waypoints.size(); i++) {
            double best = 1e9;
            int bi = -1;
            for (int j = 0; j < (int)waypoints.size(); j++)
              if (!used[j] && pdist(cur, waypoints[j]) < best) {
                best = pdist(cur, waypoints[j]);
                bi = j;
              }
            if (bi >= 0) {
              sorted.push_back(waypoints[bi]);
              cur = waypoints[bi];
              used[bi] = true;
            }
          }
          waypoints = sorted;
          current_idx = 0;
          ema_scan = scan;
          last_px = px;
          last_py = py;
          initialized = true;
          cout << "[INIT] " << waypoints.size() << " waypoints." << endl;
        }

        // stop if all done

        if (all_done) {
          a = 0;
          steer = 0;
          return;
        }

        // EMA + DYNAMIC WEIGHT given to obstacles that are approaching
        // (momentum)
        //   ema = 0.8*ema + 0.2*new_value  (α=0.2)
        // momentum = prev_ema - current_raw ML Logic: The change in EMA
        // indicates if something is approaching (positive) or departing
        // (negative) positive momentum = scan shrinking = obstacle approaching
        // boost weight for approaching, relax for departing

        if ((int)ema_scan.size() != rays)
          ema_scan = scan;
        vector<double> dyn_w(rays, 1.0);
        for (int i = 0; i < rays; i++) {
          double prev_ema = ema_scan[i];
          ema_scan[i] = 0.8 * ema_scan[i] + 0.2 * scan[i];
          double momentum = prev_ema - scan[i]; // + = approaching
          if (momentum > 0.008)
            dyn_w[i] = 1.0 + momentum * 15.0;
          else if (momentum < -0.008)
            dyn_w[i] = max(0.5, 1.0 + momentum * 4.0);
        }

        double moved = pdist({px, py}, {last_px, last_py});
        stuck_count =
            (moved < 0.0003) ? stuck_count + 1 : max(0, stuck_count - 2);
        last_px = px;
        last_py = py;

        if (escape_mode > 0) {
          escape_mode--;
          double t = 1.0 - (double)escape_mode / 50.0;
          steer = (ftype)(-1.0 + 2.0 * t);
          a = -0.15f;
          prev_steer = steer;
          prev_a = a;
          if (escape_mode == 0)
            stuck_count = 0;
          return;
        }
        if (stuck_count > 200) {
          cout << "[STUCK] escape wp " << current_idx << endl;
          escape_mode = 50;
          stuck_count = 0;
          if (current_idx < (int)waypoints.size() - 1)
            current_idx++;
          return;
        }

        // Adv Waypoint logic:
        // If close enough to current waypoint, advance to next
        if (waypoints.empty()) {
          a = 0.2f;
          steer = 0;
          return;
        }

        // Timeout: force advance if circling same waypoint too long
        if (current_idx != last_wp) {
          wp_timer = 0;
          last_wp = current_idx;
        } else
          wp_timer++;
        if (wp_timer > 600 && current_idx < (int)waypoints.size() - 1) {
          current_idx++;
          wp_timer = 0;
          stuck_count = 0;
          blocked_cd = 0;
          cout << "[TIMEOUT] wp->" << current_idx << endl;
        }

        // Larger reach radius for last waypoint
        double reach = (current_idx == (int)waypoints.size() - 1) ? 0.25 : 0.15;

        while (current_idx < (int)waypoints.size() &&
               pdist(pos, waypoints[current_idx]) < reach) {
          current_idx++;
          stuck_count = 0;
          blocked_cd = 0;
          wp_timer = 0;
          if (current_idx >= (int)waypoints.size()) {
            all_done = true;
            cout << "[DONE] All waypoints visited!" << endl;
            a = 0;
            steer = 0;
            return;
          }
          cout << "[WP] -> " << current_idx << "/" << waypoints.size() << endl;
        }

        // Blocked waypoint logic: if close to waypoint but obstacle in the way,
        // skip

        if (blocked_cd > 0)
          blocked_cd--;
        if (blocked_cd == 0 && current_idx < (int)waypoints.size()) {
          auto &wp = waypoints[current_idx];
          double ta = atan2(wp.second - py, wp.first - px);
          double mind = 1e9;
          for (int i = 0; i < rays; i++) {
            double ra = 2.0 * M_PI * i / rays + yaw;
            double diff = ra - ta;
            while (diff > PI)
              diff -= 2 * PI;
            while (diff < -PI)
              diff += 2 * PI;
            if (fabs(diff) < PI / 4)
              mind = min(mind, ema_scan[i]);
          }
          if (mind < 0.08 && pdist(pos, wp) < 0.2 &&
              current_idx < (int)waypoints.size() - 1) {
            current_idx++;
            blocked_cd = 80;
            stuck_count = 0;
            cout << "[BLOCKED] skip " << current_idx << endl;
          }
        }

        // stop and wait if obstacle extremely close (potentially dynamic
        // obstacle) to let it pass, avoid getting stuck If obstacle extremely
        // close just stop let dynamic obstacle pass

        if (min_dist < 0.06) {
          a = 0;
          steer = (ftype)(prev_steer * 0.5);
          prev_steer = steer;
          return;
        }

        // Pure pursuit

        double Ld = max(0.15, min(0.28, 0.15 + 1.5 * fabs(spd)));
        pair<double, double> target =
            waypoints[min(current_idx, (int)waypoints.size() - 1)];
        for (int i = current_idx; i < (int)waypoints.size(); i++)
          if (pdist(pos, waypoints[i]) > Ld) {
            target = waypoints[i];
            break;
          }

        double tdx = target.first - px, tdy = target.second - py;
        double local_y = sin(-yaw) * tdx + cos(-yaw) * tdy;
        double goal_steer = (2.0 * local_y) / (Ld * Ld);
        goal_steer = max(-1.0, min(1.0, goal_steer));

        // Obstacle avoiding
        // ray 0 = forward, mid = backward
        // i < mid = right half, i >= mid = left half
        // frontal boost: ray 0 (forward) gets 2x weight

        const double DANGER = 0.22;
        double left_threat = 0, right_threat = 0;
        double min_d = 1e9;
        int mid = rays / 2;

        for (int i = 0; i < rays; i++) {
          double d = ema_scan[i]; // EMA = smooth, stable
          min_d = min(min_d, d);

          if (d < DANGER) {
            double w = (DANGER - d) / DANGER;
            w = w * w * 3.0;
            w *= dyn_w[i]; // EMA momentum boost
            // frontal boost: cos peaks at i=0 (forward)
            w *= (1.0 + max(0.0, cos(2.0 * PI * i / rays)));
            if (i < mid)
              right_threat += w;
            else
              left_threat += w;
          }
        }

        double avoid = left_threat - right_threat;

        // Danger signal = how close the nearest obstacle is (0 to 1) - used to
        // modulate between goal-steer and avoid, and to reduce speed when in
        // danger danger_rate = derivative of danger signal same as momentum
        // indicator

        double danger =
            (min_d < DANGER) ? min(1.0, (DANGER - min_d) / DANGER * 2.0) : 0.0;

        double danger_rate = danger - prev_danger;
        prev_danger = danger;
        double anticipate = max(0.0, danger_rate * 4.0);

        double raw_steer = (1.0 - danger) * goal_steer + danger * avoid * 2.0;
        raw_steer = max(-1.0, min(1.0, raw_steer));

        steer = (ftype)(0.72 * prev_steer + 0.28 * raw_steer);
        prev_steer = steer;

        double spd_target = 0.25 * (1.0 - 0.6 * fabs((double)steer)) *
                            (1.0 - 0.7 * danger) * (1.0 - anticipate);
        spd_target = max(0.02, spd_target);
        a = (ftype)(0.85 * prev_a + 0.15 * spd_target);
        prev_a = a;

        // 11. Boundary check to avoid going out of bounds - steer back in if
        // near edge

        if (fabs(px) > 0.85 || fabs(py) > 0.85) {
          double cx = -px, cy = -py;
          double cl = sqrt(cx * cx + cy * cy);
          if (cl > 1e-5) {
            cx /= cl;
            cy /= cl;
          }
          double ly = sin(-yaw) * cx + cos(-yaw) * cy;
          steer =
              (ftype)(0.72 * prev_steer + 0.28 * max(-1.0, min(1.0, 2.0 * ly)));
          prev_steer = steer;
          a = min(a, 0.07f);
        }

        static int frame = 0;
        if (frame++ % 200 == 0)
          cout << "[f=" << frame << "] pos=(" << px << "," << py
               << ") wp=" << current_idx << "/" << waypoints.size()
               << " d=" << danger << " L=" << left_threat
               << " R=" << right_threat << " stuck=" << stuck_count << endl;
      };

  array<agent, playercount> myagents;
  for (int i = 0; i < playercount; i++)
    myagents[i] = myagent;

  simulationinstance s(myagents, 120);

  // Dynamic obstacles

  int num_obs = s.mp.size() - 1;
  for (int i = 0; i < num_obs; i++) {
    point c = {0, 0};
    for (auto &p : s.mp[i]) {
      c.x += p.x;
      c.y += p.y;
    }
    c.x /= s.mp[i].size();
    c.y /= s.mp[i].size();
    float phase = 2.0f * M_PI * i / num_obs;

    if (i % 3 == 0) {
      float amp = 0.06f + 0.02f * (i % 4), freq = 0.7f + 0.2f * (i % 5);
      s.movementspecifier[i] = [c, amp, freq, phase](shape &obs,
                                                     const ftype &t) {
        point cur = {0, 0};
        for (auto &p : obs) {
          cur.x += p.x;
          cur.y += p.y;
        }
        cur.x /= obs.size();
        cur.y /= obs.size();
        float tx = c.x + amp * sin(freq * t + phase);
        for (auto &p : obs)
          p.x += tx - cur.x;
      };
    } else if (i % 3 == 1) {
      float amp = 0.06f + 0.02f * (i % 3), freq = 0.6f + 0.2f * (i % 4);
      s.movementspecifier[i] = [c, amp, freq, phase](shape &obs,
                                                     const ftype &t) {
        point cur = {0, 0};
        for (auto &p : obs) {
          cur.x += p.x;
          cur.y += p.y;
        }
        cur.x /= obs.size();
        cur.y /= obs.size();
        float ty = c.y + amp * sin(freq * t + phase);
        for (auto &p : obs)
          p.y += ty - cur.y;
      };
    } else {
      float amp = 0.05f + 0.02f * (i % 3), freq = 0.5f + 0.2f * (i % 4);
      s.movementspecifier[i] = [c, amp, freq, phase](shape &obs,
                                                     const ftype &t) {
        point cur = {0, 0};
        for (auto &p : obs) {
          cur.x += p.x;
          cur.y += p.y;
        }
        cur.x /= obs.size();
        cur.y /= obs.size();
        float tx = c.x + amp * cos(freq * t + phase);
        float ty = c.y + amp * sin(freq * t + phase);
        for (auto &p : obs) {
          p.x += tx - cur.x;
          p.y += ty - cur.y;
        }
      };
    }
  }

  s.humanmode = false;
  s.visualmode = true;
  s.run();
  return 0;
}
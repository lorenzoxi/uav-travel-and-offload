#include "gurobi_c++.h"
#include <iostream>
#include <vector>
#include <set>
#include <tuple>
#include <map>
#include "read_data.h"
#include "neighborhood.h"
#include "SpreadingFactor.h"
#include "SF.h"
#include "LoRa.h"
#include <regex>


using namespace std;
namespace fs = std::filesystem;

int main() {

    /**
     * DATA READING
     */
    const std::string PATH = "../instances/";

    std::vector<std::string> instances5 = {
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_d",
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_c",
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_j",
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_b",
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_e",
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_g",
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_i",
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_h",
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_f",
            PATH + "5/instance_2km_500m_sensors5__20250721_151107_a"
    };

    std::vector<std::string> instances25 = {
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_b",
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_e",
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_d",
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_c",
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_j",
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_h",
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_f",
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_a",
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_g",
            PATH + "25/instance_2km_500m_sensors25__20250721_151117_i"
    };

    std::vector<std::string> instances50 = {
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_b",
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_e",
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_d",
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_c",
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_j",
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_h",
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_f",
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_a",
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_g",
            PATH + "50/instance_2km_500m_sensors50__20250721_151128_i"
    };

    std::vector<std::string> instances100 = {
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_d",
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_c",
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_j",
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_b",
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_e",
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_g",
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_i",
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_h",
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_f",
            PATH + "100/instance_2km_500m_sensors100__20250721_151140_a"
    };

    std::vector<std::string> instances200 = {
            PATH + "200/instance_2km_500m_sensors200__20250721_151353_f",
            PATH + "200/instance_2km_500m_sensors200__20250721_151353_h",
            PATH + "200/instance_2km_500m_sensors200__20250721_151353_i",
            PATH + "200/instance_2km_500m_sensors200__20250721_151353_g",
            PATH + "200/instance_2km_500m_sensors200__20250721_151353_e",
            PATH + "200/instance_2km_500m_sensors200__20250721_151353_b",
            PATH + "200/instance_2km_500m_sensors200__20250721_151353_j",
            PATH + "200/instance_2km_500m_sensors200__20250721_151353_c",
            PATH + "200/instance_2km_500m_sensors200__20250721_151353_d",
            PATH + "200/instance_2km_500m_sensors200__20250721_151352_a",
    };

    std::vector<std::string> instances250 = {
            PATH + "250/instance_2km_500m_sensors250__20250723_095252_j",
            PATH + "250/instance_2km_500m_sensors250__20250723_095251_b",
            PATH + "250/instance_2km_500m_sensors250__20250723_095251_e",
            PATH + "250/instance_2km_500m_sensors250__20250723_095251_d",
            PATH + "250/instance_2km_500m_sensors250__20250723_095251_c",
            PATH + "250/instance_2km_500m_sensors250__20250723_095252_h",
            PATH + "250/instance_2km_500m_sensors250__20250723_095252_f",
            PATH + "250/instance_2km_500m_sensors250__20250723_095252_g",
            PATH + "250/instance_2km_500m_sensors250__20250723_095252_i",
            PATH + "250/instance_2km_500m_sensors250__20250723_095251_a",
    };

    std::vector<std::string> instances500 = {
            PATH + "500/instance_2km_500m_sensors500__20250723_165201_a",
            PATH + "500/instance_2km_500m_sensors500__20250723_165203_e",
            PATH + "500/instance_2km_500m_sensors500__20250723_165204_i",
            PATH + "500/instance_2km_500m_sensors500__20250723_165204_g",
            PATH + "500/instance_2km_500m_sensors500__20250723_165202_d",
            PATH + "500/instance_2km_500m_sensors500__20250723_165202_c",
            PATH + "500/instance_2km_500m_sensors500__20250723_165204_h",
            PATH + "500/instance_2km_500m_sensors500__20250723_165205_j",
            PATH + "500/instance_2km_500m_sensors500__20250723_165201_b",
            PATH + "500/instance_2km_500m_sensors500__20250723_165203_f",

    };
    

    auto instances_vec = {
            instances200,
    };

    for (auto &instances: instances_vec) {
        for (auto &instance: instances) {

            std::filesystem::path path(instance);
            std::string last_crumb = path.filename().string();
            std::regex number_regex(R"((\d+))");
            std::smatch match;
            std::vector<int> numbers;
            auto it = last_crumb.cbegin();
            auto end = last_crumb.cend();

            while (std::regex_search(it, end, match, number_regex)) {
                numbers.push_back(std::stoi(match[1]));
                it = match.suffix().first;
            }
            int km = numbers[0];
            int grid = numbers[1];
            int n_sensors = numbers[2];

            std::string instance_file_path = instance + "/instance_" +
                                             std::to_string(km) + "_" +
                                             std::to_string(grid) + "_" +
                                             std::to_string(n_sensors) + ".json";
            std::string energy_file_path = instance + "/energy_matrix_instance_" +
                                           std::to_string(km) + "_" +
                                           std::to_string(grid) + "_" +
                                           std::to_string(n_sensors) + ".json";

            std::cout
                    << "------------------------------------------------------------------------------------------"
                    << std::endl;
            std::cout << "Processing instance: " << last_crumb << "(" << instance_file_path << ")"
                      << std::endl;
            std::cout
                    << "------------------------------------------------------------------------------------------"
                    << std::endl;

            //get the last char of the last crumb
            std::string last_char = last_crumb.substr(last_crumb.size() - 1);

            std::string solution_folder = "../solutions/" + std::to_string(n_sensors);
            if (!fs::exists(solution_folder)) {
                try {
                    fs::create_directories(solution_folder);
                    std::cout << "Created directory: " << solution_folder << std::endl;
                } catch (const fs::filesystem_error &e) {
                    std::cerr << "Error creating directory: " << e.what() << std::endl;
                    return 1;
                }
            } else {
                std::cout << "Directory already exists: " << solution_folder << std::endl;
            }

            // Check if the instance file exists
            if (!std::filesystem::exists(instance_file_path)) {
                std::cerr << "Instance file does not exist: " << instance_file_path << std::endl;
                throw std::runtime_error("Instance file not found");
            }

            // Check if the energy file exists
            if (!std::filesystem::exists(energy_file_path)) {
                std::cerr << "Energy file does not exist: " << energy_file_path << std::endl;
                throw std::runtime_error("Energy file not found");
            }


            vector<Sensor> S = load_sensors(instance_file_path);                // Sensors
            std::vector<SpreadingFactor> A = SF125;                                   // Spreading Factors
            int start = 0;
            double H = 88;
            double bigM = 1e6;
            map<pair<int, int>, double> E = load_energy_map(energy_file_path);  // E: energy map
            map<int, double> D, P;

            for (const auto &sensor: S) {
                D[sensor._id] = sensor._id;
            }

            for (auto a: A) {
                P[a._id] = a.Kbps;
            }

            std::vector<double> PW_values = {
                    0,
                    1.5849,   // 2 dBm
                    3.1623,   // 5 dBm
                    5.0119,   // 7 dBm
                    10.0000,  // 10 dBm
                    15.8489,  // 12 dBm
                    25.1189,  // 14 dBm
                    39.8107,  // 16 dBm
                    50.1187   // 17 dBm
            };
            double maxPW = PW_values.back();
            unsigned int M_size = S.size();

            double c_1 = 2.5;           // coefficient for the first term in the transmission power constraint
            double c_2 = 10;            // coefficient for the second term in the transmission power constraint
            double alpha = 27.66;       // coefficient for the transmission power constraint
            double UAV_height = 10000;
            double sensors_height = 0;

            try {
                GRBEnv env = GRBEnv(true);
                env.start();
                GRBModel model = GRBModel(env);
                // Set time limit
                model.set(GRB_DoubleParam_TimeLimit, 60 * 360);
                // Set MIPGap to 1e-3 (in %: 0.1%)
                model.set(GRB_DoubleParam_MIPGap, 1e-3);

                // Set log
                model.set(GRB_StringParam_LogFile,
                          solution_folder + "/" + last_char + "_log.log");


                // Decision variables
                map<pair<int, int>, GRBVar> x;              // x[i][j] bool, path from i to j
                map<pair<int, int>, GRBVar> f;              // f[i][j] int, flow from i to j
                map<tuple<int, int, int>, GRBVar> sigma;    // sigma[t][i][c][a]
                map<int, GRBVar> h;                         // h[t]: hoovering time (in sec) at slot t
                map<std::tuple<int, int, int>, GRBVar> d;   // d[i][j]: bool, true if sensor i offload to j (i.e. when UAV is in position j)
                map<std::tuple<int, int, int>, GRBVar> w;
                map<pair<int, int>, GRBVar> pw;
                map<int, GRBVar> v;

                std::vector<std::vector<GRBVar>> pw_vars = {};
                for (auto p: S) {
                    // For each sensor p, create variables for each other sensor q
                    h[p._id] = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_INTEGER,
                                            "h_" + to_string(p._id));
                    v[p._id] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY,
                                            "v_" + to_string(p._id));

                    pw_vars.push_back({});
                    for (auto q: S) {
                        x[{p._id, q._id}] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY,
                                                         "x_" + to_string(p._id) + "_" +
                                                         to_string(q._id));

                        f[{p._id, q._id}] = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_INTEGER,
                                                         "f_" + to_string(p._id) + "_" +
                                                         to_string(q._id));

                        pw[{p._id, q._id}] = model.addVar(0.0, maxPW, 1.0, GRB_CONTINUOUS,
                                                          "pw_" + to_string(p._id) + "_" +
                                                          to_string(q._id));

                        for (auto sf: A) {
                            d[{p._id, q._id, sf._id}] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY,
                                                                     "d_" + to_string(p._id) + "_" +
                                                                     to_string(q._id) + "_" +
                                                                     to_string(sf._id));
                            w[{p._id, q._id, sf._id}] = model.addVar(0.0, GRB_INFINITY, 1.0,
                                                                     GRB_CONTINUOUS,
                                                                     "w_" + to_string(p._id) + "_" +
                                                                     to_string(q._id) + "_" +
                                                                     to_string(sf._id));
                        }
                    }
                }

                /** ======== TSP CONSTRAINTS ======== **/
                // (tsp) Degree constraints (excluding depot)
                for (int i = 0; i < S.size(); ++i) {
                    GRBLinExpr out_degree = 0, in_degree = 0;
                    auto id_i = S[i]._id;
                    for (int j = 0; j < S.size(); ++j) {
                        auto id_j = S[j]._id;
                        out_degree += x[{id_i, id_j}];
                        in_degree += x[{id_j, id_i}];
                    }
                    model.addConstr(out_degree <= v[id_i], "out_degree_" + to_string(id_i));
                    model.addConstr(in_degree <= v[id_i], "in_degree_" + to_string(id_i));
                }

                // (tsp) No self-loops
                for (auto p: S) {
                    model.addConstr(x[{p._id, p._id}] == 0, "no_self_arc_" + to_string(p._id));
                    model.addConstr(f[{p._id, p._id}] == 0, "no_self_flow_" + to_string(p._id));
                }

                // (tsp) sum x_(start)j <= 1 for each j in M\{start}
                GRBLinExpr sum_x_start = 0;
                for (auto q: S) {
                    if (q._id != start) {
                        sum_x_start += x[{start, q._id}];
                    }
                }
                model.addConstr(sum_x_start <= 1, "sum_x_start");

                // start at the depot
                model.addConstr(v[{start}] == 1, "v_start_one");

                // (tsp) Ensure that what exit the depot is what enters it
                GRBLinExpr depot_out_edges = 0;
                for (auto q: S) {
                    if (q._id != start) {
                        depot_out_edges += x[{start, q._id}];
                    }
                }
                GRBLinExpr depot_in_edges = 0;
                for (auto p: S) {
                    if (p._id != start) {
                        depot_in_edges += x[{p._id, start}];
                    }
                }
                model.addConstr(depot_out_edges == depot_in_edges, "depot_balance");

                /** ======== FLOW SEC CONSTRAINTS Gavish & Graves SEC) ======== **/
                // (c-flow)  (sum f_ik = for each i in M) - (sum f_kj = for each j in M) = 1 for each k in M\{start}
                for (auto k: S) {
                    if (k._id != start) {
                        GRBLinExpr sum_flow_in = 0;
                        for (auto i: S) {
                            sum_flow_in += f[{i._id, k._id}];
                        }
                        GRBLinExpr sum_flow_out = 0;
                        for (auto i: S) {
                            if (i._id != k._id) {
                                sum_flow_out += f[{k._id, i._id}];
                            }
                        }
                        model.addConstr((sum_flow_in - sum_flow_out) == v[k._id],
                                        "flow_" + to_string(k._id));
                    }
                }

                // (c-flow) f_ij <= |V_S|x_ij for each i in  and j in V_S
                for (auto p: S) {
                    for (auto q: S) {
                        if (p._id != q._id) {
                            model.addConstr(f[{p._id, q._id}] <= (S.size() - 1) * x[{p._id, q._id}],
                                            "flow_bound_" + to_string(p._id) + "_" +
                                            to_string(q._id));
                        }
                    }
                }

                // (c-flow) f_ij >= 0 for each i in V and j in M   //TODO: togliere
                for (auto p: S) {
                    for (auto q: S) {
                        if (p._id != q._id) {
                            model.addConstr(f[{p._id, q._id}] >= 0,
                                            "flow_non_negative_" + to_string(p._id) + "_" +
                                            to_string(q._id));
                        }
                    }
                }

                // (c-flow) f_i0 = 0 for each i in M
                for (auto p: S) {
                    if (p._id != start) {
                        model.addConstr(f[{p._id, start}] == 0,
                                        "flow_to_depot_" + to_string(p._id));
                    }
                }

                /** ======== OFFLOADING CONSTRAINTS ======== **/
                // sum d_ij = 1 for each i in S
                for (auto p: S) {
                    GRBLinExpr sum_d = 0;
                    for (auto q: S) {
                        for (auto sf: A) {
                            sum_d += d[{q._id, p._id, sf._id}];
                        }
                    }
                    model.addConstr(sum_d == 1, "offloading_sum_" + std::to_string(p._id));
                }

                /** ======== PW ND SIGNAL CONSTRAINTS ======== **/
                std::map<std::tuple<int, int, int>, GRBVar> y;
                for (auto p: S) {
                    for (auto q: S) {
                        GRBLinExpr pw_expr = 0;
                        GRBLinExpr y_sum = 0;
                        for (int k = 0; k < PW_values.size(); ++k) {
                            y[{p._id, q._id, k}] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY,
                                                                "y_" + to_string(p._id) + "_" +
                                                                to_string(q._id) + "_" +
                                                                to_string(k));
                            pw_expr += y[{p._id, q._id, k}] * PW_values[k];
                            y_sum += y[{p._id, q._id, k}];
                        }
                        model.addConstr(pw[{p._id, q._id}] == pw_expr,
                                        "pw_expr_" + std::to_string(p._id) + "_" +
                                        std::to_string(q._id));
                        model.addConstr(y_sum == 1, "pw_y_sum_" + std::to_string(p._id) + "_" +
                                                    std::to_string(q._id));
                    }
                }

                //if sum djis == 0 => pw_ij = 0
                for (auto p: S) {
                    for (auto q: S) {
                        GRBLinExpr sum_d = 0;
                        for (auto sf: A) {
                            sum_d += d[{p._id, q._id, sf._id}];
                        }

                        model.addConstr(pw[{p._id, q._id}] <= bigM * sum_d),
                                "pw_zero_if_no_offloading_" + std::to_string(p._id);
                    }
                }

                // for each i in S, sum dijs <= 8 (max demodulation constraint)
                for (auto p: S) {
                    GRBLinExpr sum_d = 0;
                    for (auto q: S) {
                        for (auto sf: A) {
                            sum_d += d[{p._id, q._id, sf._id}];
                        }
                    }
                    model.addConstr(sum_d <= 8, "sum_dijs_" + std::to_string(p._id));
                }


                // signal strength constraint
                for (auto p: S) {
                    for (auto q: S) {
                        for (auto sf: A) {
                            double distance_in_m =
                                    ::distance(p.lat, p.lon, p.z, q.lat, q.lon, UAV_height);

                            double K = pow(10.0, (-c_1 * sf._id + c_2 - alpha) /
                                                 10.0);

                            auto var = K * distance_in_m;
                            std::cout << "Var: " << var << std::endl;

                            double tight_M = *max_element(PW_values.begin(), PW_values.end());

                            model.addConstr(
                                    pw[{p._id, q._id}] >=
                                    K * distance_in_m - tight_M * (1 - d[{p._id, q._id, sf._id}]),
                                    "signal_strength_cns_" + std::to_string(p._id) + "_" +
                                    std::to_string(q._id) + "_" + std::to_string(sf._id));
                        }
                    }
                }

                /** ======== HOOVERING CONSTRAINTS ======== **/
                for (auto s1: S) {
                    for (auto s2: S) {
                        for (auto sf: A) {
                            double toa = get_time_on_air(s1.payload, sf._id,
                                                         sf.bandwidth * 1000);
                            /* std::cout << "TOA for sensor " << s1._id << " to " << s2._id
                                      << " with SF " << sf._id << ": " << toa << " seconds"
                                      << std::endl; */
                            toa = toa < 1 ? 1 : toa;
                            model.addConstr(h[s1._id] >= d[{s1._id, s2._id, sf._id}] * toa,
                                            "hoovering_time_" + to_string(s1._id) + "_" +
                                            to_string(s2._id) + "_" + to_string(sf._id));
                        }
                    }
                }

                //if v[s._id] == 0 then h[s._id] = 0
                for (auto s: S) {
                    model.addConstr(h[s._id] <= bigM * v[s._id],
                                    "hoovering_time_bound_" + to_string(s._id));
                }

                // if v[s._id] == 0 then sum d_ijs = 0
                for (auto s: S) {
                    GRBLinExpr sum_d = 0;
                    for (auto q: S) {
                        for (auto sf: A) {
                            sum_d += d[{s._id, q._id, sf._id}];
                        }
                    }
                    model.addConstr(sum_d <= bigM * v[s._id],
                                    "sum_dijs_bound_" + to_string(s._id));
                }

                /** ======== LINERIZATION OF (pw_i)*(d_ijs) ======== **/
                for (auto p: S) {
                    for (auto q: S) {
                        for (auto a: A) {
                            model.addConstr(w[{p._id, q._id, a._id}] <= pw[{p._id, q._id}],
                                            "lin1_pw_d_" + to_string(p._id) + "_");
                            model.addConstr(
                                    w[{p._id, q._id, a._id}] <= maxPW * d[{p._id, q._id, a._id}],
                                    "lin2_pw_d_" + to_string(p._id) + "_");
                            model.addConstr(w[{p._id, q._id, a._id}] >=
                                            pw[{p._id, q._id}] -
                                            (1 - d[{p._id, q._id, a._id}]) * maxPW,
                                            "lin3_pw_d_" + to_string(p._id) + "_");

                        }
                    }
                }

                /** ======== TOA CONSTRAINTS ======== **/
                // sum dijs * toa <= 36
                for (auto p: S) {
                    GRBLinExpr sum_d_toa = 0;
                    for (auto q: S) {
                        for (auto sf: A) {
                            double toa = get_time_on_air(p.payload * 10, sf._id,
                                                         sf.bandwidth * 1000);
                            toa = toa < 1 ? 1 : toa; // Ensure toa is at least 1 second
                            model.addConstr(d[{p._id, q._id, sf._id}] * toa <= 36,
                                            "d_toa_lorawan_" + std::to_string(p._id) + "_" +
                                            std::to_string(q._id) + "_" + std::to_string(sf._id));
                        }
                    }
                }


                /** ======== OBJECTIVE FUNCTION ======== **/
                GRBLinExpr cost_movement = 0;
                for (auto p: S) {
                    for (auto q: S) {
                        if (p._id != q._id) {
                            cost_movement += x[{p._id, q._id}] * E[{p._id, q._id}];
                        }
                    }
                }

                GRBLinExpr cost_hoovering = 0;
                for (auto s: S) {
                    cost_hoovering +=
                            h[s._id] * H; // Assuming 88 is the cost per second of hoovering
                }

                GRBLinExpr cost_trasmission = 0;
                for (auto p: S) {
                    for (auto q: S) {
                        for (auto sf: A) {
                            double toa = get_time_on_air(p.payload, sf._id, sf.bandwidth * 1000);
                            toa = toa < 1 ? 1 : toa;
                            cost_trasmission += (0.001 * w[{p._id, q._id, sf._id}]) * toa;
                        }
                    }
                }

                //Add three variables to the model for the costs
                GRBVar ofM = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "ofT");
                GRBVar ofH = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "ofH");
                GRBVar ofTx = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "ofTx");

                model.addConstr(ofM == cost_movement, "ofM");
                model.addConstr(ofH == cost_hoovering, "ofH");
                model.addConstr(ofTx == cost_trasmission, "ofTx");

                model.addConstr(ofM + ofTx <= 600000);

                model.setObjective(
                        ofM + ofH + ofTx,
                        GRB_MINIMIZE
                );

                std::string sol_json = solution_folder + "/" + last_char + "_solution.json";
                std::string sol_lp = solution_folder + "/" + last_char + "_model.lp";
                std::string sol_sol = solution_folder + "/" + last_char + "_solution.sol";

                model.optimize();
                model.write(sol_lp);
                model.write(sol_json);
                model.write(sol_sol);

                cout << "Optimal objective value: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

            } catch (GRBException &e) {
                cerr << "Error code = " << e.getErrorCode() << endl;
                cerr << e.getMessage() << endl;
            } catch (...) {
                cerr << "Exception during optimization" << endl;
            }
        }
    }
    return 0;
}
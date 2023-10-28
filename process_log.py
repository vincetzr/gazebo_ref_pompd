import re
import sys
import fileinput
from statistics import NormalDist

from pathlib import Path


def get_std_and_ci(data, confidence=0.95):
    if len(data) < 30:
        print("WARNING: Sample size is less than 30")
    dist = NormalDist.from_samples(data)
    z = NormalDist().inv_cdf((1 + confidence) / 2.)
    h = dist.stdev * z / ((len(data) - 1) ** .5)
    return dist.stdev, h


def go_n_lines(fp, lines=1):
    ln = None
    for i in range(lines):
        ln = fp.readline()
    return ln


def parse_reward_str(s_line):
    processed_l = s_line.replace("\n", "").replace(" ", "").replace("[", "").replace("]", "")
    if len(processed_l) > 0:
        return [float(x) for x in processed_l.split(",")]
    else:
        return []


def get_list_print(ll):
    pl = []
    for (i, v) in enumerate(ll):
        pl.append(f"{i}: {v}")
    return pl


def print_reward_and_success(reward_list, success_list, process_list, trial_list):
    if len(reward_list) != len(success_list):
        raise ValueError(f"Reward and success list is not mathing {len(reward_list)} {len(success_list)}")
    elif len(reward_list) != len(process_list):
        raise ValueError(f"Reward and process list is not mathing {len(reward_list)} {len(process_list)}")
    elif len(reward_list) != len(trial_list):
        raise ValueError(f"Reward and trial list is not mathing {len(reward_list)} {len(trial_list)}")

    print("process_id, trial_id, reward, success")
    for i in range(len(reward_list)):
        print(f"{process_list[i]} {int(trial_list[i])} {reward_list[i]} {int(success_list[i])}")


def collect_summary_data(
        fp,
        trials_without_errors,
        successful_trials,
        avg_cum_disc_reward,
        max_cum_disc_reward,
        min_cum_disc_reward,
        errors,
        reward_discounted,
        success_list,
        trial_list,
        total_trials):
    line = go_n_lines(fp)
    trials_without_e = int(str(line[22:]).split()[0])
    trials_without_errors.append(trials_without_e)
    print("Trials without errors: ", trials_without_e)
    line = go_n_lines(fp)
    successful_t = int(str(line[18:]).split()[0])
    successful_trials.append(successful_t)
    print("Successful trials: ", successful_t)
    if trials_without_e > 0:
        line = go_n_lines(fp)
        avg_cum_disc_r = float(str(line[21:].split()[0]))
        avg_cum_disc_reward.append(avg_cum_disc_r)
        print("avg cum disc reward: ", avg_cum_disc_r)
        line = go_n_lines(fp)
        max_cum_disc_r = float(str(line[21:].split()[0]))
        max_cum_disc_reward.append(max_cum_disc_r)
        print("max cum disc reward: ", max_cum_disc_r)
        line = go_n_lines(fp)
        min_cum_disc_r = float(str(line[21:].split()[0]))
        min_cum_disc_reward.append(min_cum_disc_r)
        print("min cum disc reward: ", min_cum_disc_r)
        line = go_n_lines(fp, 3)
    else:
        line = go_n_lines(fp, 6)
    err = str(line[8:]).replace("\n", "")
    errors.append(err)
    print("Errors: ", err)
    if trials_without_e > 0:
        line = go_n_lines(fp)
        r_list = parse_reward_str(line[24:])
        reward_discounted.extend(r_list)
        print("Reward discounted list: ", r_list)
        line = go_n_lines(fp)
        s_list = parse_reward_str(line[14:])
        success_list.extend(s_list)
        print("Success list: ", s_list)
        line = go_n_lines(fp)
        t_list = parse_reward_str(line[12:])
        trial_list.extend(t_list)
        print("Trial list: ", s_list)
        line = go_n_lines(fp)
    else:
        line = go_n_lines(fp, 4)
    total_t = int(str(line[14:]).split()[0])
    total_trials.append(total_t)
    print("total trials: ", total_t)


def print_aggregated_summary(
        trials_without_errors,
        successful_trials,
        avg_cum_disc_reward,
        max_cum_disc_reward,
        min_cum_disc_reward,
        errors,
        reward_discounted,
        success_list,
        trial_list,
        process_list,
        total_trials):
    print("Total trials without errors: ", sum(trials_without_errors))
    print("Total successful trials    : ", sum(successful_trials))
    if len(reward_discounted) > 2:
        print("Discounted reward min: ", min(reward_discounted))
        print("Discounted reward max: ", max(reward_discounted))
        print("Discounted reward avg: ", sum(reward_discounted) / len(reward_discounted))
        std_and_ci = get_std_and_ci(reward_discounted)
        print("Discounted reward std: ", std_and_ci[0])
        print("Discounted reward 95% CI: ±", std_and_ci[1])
    elif len(avg_cum_disc_reward) > 0:
        print("Discounted reward min: ", min(min_cum_disc_reward))
        print("Discounted reward max: ", max(max_cum_disc_reward))
        print("Discounted reward avg: ", sum(avg_cum_disc_reward) / len(avg_cum_disc_reward))
    print("Total trials  : ", sum(total_trials))
    if len(success_list) > 0:
        print("Success rate (from no error) : ", sum(success_list) / len(success_list))
    else:
        print("Success rate (from no error) : 0")
    print("Success rate (from total)    : ", sum(success_list) / sum(total_trials))
    print("\n-----------Trial success data------------")
    print_reward_and_success(reward_discounted, success_list, process_list, trial_list)
    print("\n\n-----------------------------------------\n\n")
    print("Errors                     : ", errors)


if __name__ == '__main__':
    print("Processing logs")

    working_dir = Path.cwd()
    log_dir = (working_dir / sys.argv[1]).resolve()
    # file_name_match = sys.argv[2]

    print(f"Working dir: {working_dir}")
    print(f"Log dir    : {log_dir}")
    # print(f"Log Name match: {log_dir}")

    # regex = r".+(" + file_name_match + ")\w+(\.out)"
    regex = r".+\w+(\.out)"

    files_in_log_dir = [entry for entry in log_dir.iterdir() if entry.is_file() and bool(re.search(regex, entry.name))]

    log_codes = set()
    log_code_regex = r"Process_(.+)_[\d]{1,2}_([MTWFS].+).out"
    for file in files_in_log_dir:
        result = re.search(log_code_regex, file.name)
        log_codes.add((result.group(1), result.group(2)))

    trials_without_errors_1 = []
    successful_trials_1 = []
    errors_1 = []
    avg_cum_disc_reward_1 = []
    max_cum_disc_reward_1 = []
    min_cum_disc_reward_1 = []
    reward_discounted_1 = []
    success_list_1 = []
    trial_list_1 = []
    process_list_1 = []
    total_trials_1 = []

    trials_without_errors_2 = []
    successful_trials_2 = []
    errors_2 = []
    avg_cum_disc_reward_2 = []
    max_cum_disc_reward_2 = []
    min_cum_disc_reward_2 = []
    reward_discounted_2 = []
    success_list_2 = []
    trial_list_2 = []
    process_list_2 = []
    total_trials_2 = []

    trials_without_errors_3 = []
    successful_trials_3 = []
    errors_3 = []
    avg_cum_disc_reward_3 = []
    max_cum_disc_reward_3 = []
    min_cum_disc_reward_3 = []
    reward_discounted_3 = []
    success_list_3 = []
    trial_list_3 = []
    process_list_3 = []
    total_trials_3 = []

    for file in files_in_log_dir:
        print("Processing log file : ", file.name)

        process_id = re.search(r"Process_(.+_\d+)_[MTWFS]", file.name).group(1)
        print("Process id: ", process_id)

        with file.open('r') as fp:
            line = fp.readline()
            while not line.startswith("***** RESULTS *****"):
                line = fp.readline()

            print("Results RefSolver\n")
            line = go_n_lines(fp, 3)
            collect_summary_data(
                fp,
                trials_without_errors_1,
                successful_trials_1,
                avg_cum_disc_reward_1,
                max_cum_disc_reward_1,
                min_cum_disc_reward_1,
                errors_1,
                reward_discounted_1,
                success_list_1,
                trial_list_1,
                total_trials_1
            )
            process_list_1.extend([process_id] * trials_without_errors_1[-1])

            # print("Results POMCP (unif rollout)\n")
            # line = go_n_lines(fp, 2)
            # collect_summary_data(
            #     fp,
            #     trials_without_errors_2,
            #     successful_trials_2,
            #     avg_cum_disc_reward_2,
            #     max_cum_disc_reward_2,
            #     min_cum_disc_reward_2,
            #     errors_2,
            #     reward_discounted_2,
            #     success_list_2,
            #     trial_list_2,
            #     total_trials_2
            # )
            # process_list_2.extend([process_id] * trials_without_errors_2[-1])

            print("Results POMCP (A* rollout)\n")
            line = go_n_lines(fp, 2)
            collect_summary_data(
                fp,
                trials_without_errors_3,
                successful_trials_3,
                avg_cum_disc_reward_3,
                max_cum_disc_reward_3,
                min_cum_disc_reward_3,
                errors_3,
                reward_discounted_3,
                success_list_3,
                trial_list_3,
                total_trials_3
            )
            process_list_3.extend([process_id] * trials_without_errors_3[-1])

            print("\n")

    print("-----------Log codes------------")
    for lc in log_codes:
        print("    ", lc)
    print("\n\n")

    print("------Experiment Summary--------")

    print("---Ref Solver---")
    print_aggregated_summary(
        trials_without_errors_1,
        successful_trials_1,
        avg_cum_disc_reward_1,
        max_cum_disc_reward_1,
        min_cum_disc_reward_1,
        errors_1,
        reward_discounted_1,
        success_list_1,
        trial_list_1,
        process_list_1,
        total_trials_1
    )

    print("\n\n\n")

    # print("---POMCP (unif rollout)---")
    # print_aggregated_summary(
    #     trials_without_errors_2,
    #     successful_trials_2,
    #     avg_cum_disc_reward_2,
    #     max_cum_disc_reward_2,
    #     min_cum_disc_reward_2,
    #     errors_2,
    #     reward_discounted_2,
    #     success_list_2,
    #     trial_list_2,
    #     process_list_2,
    #     total_trials_2
    # )
    #
    # print("\n\n\n")

    print("---Results POMCP (A* rollout)---")
    print_aggregated_summary(
        trials_without_errors_3,
        successful_trials_3,
        avg_cum_disc_reward_3,
        max_cum_disc_reward_3,
        min_cum_disc_reward_3,
        errors_3,
        reward_discounted_3,
        success_list_3,
        trial_list_3,
        process_list_3,
        total_trials_3
    )

    print("------Experiment End--------")

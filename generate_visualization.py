import math
import re
import sys
from enum import Enum

import matplotlib
import matplotlib.pyplot as plt

from pathlib import Path

matplotlib.use("TkAgg")


class GridMap:

    class ObjectColors:
        Obstacle = 'black'
        ObstacleNew = '#a0a0a0'   #993300
        Danger = '#FFCC33'
        Goal = 'green'
        Landmark = '#FFCCFF'
        Robot = 'red'
        Belief = 'blue'
        Start = 'black'
        Blank = 'white'

    def __init__(self, n, m, obstacles, landmarks, danger_zones, goals):
        """
        n: number of rows (x-direction)
        m: number of columns (y-direction)
        obstacles: list of tuples indicating co-ordinates of obstacles.
        landmarks: list of tuples indicating co-ordinates of landmarks.
        goals: a list of tuples indicating the co-ordinates of the goal states.
        danger_zones: a list of tuples indicating the co-ordinates of danger zones.
        """
        self.n = n
        self.m = m
        self.obstacles = obstacles
        self.obstacles_new = []
        self.landmarks = landmarks
        self.danger_zones = danger_zones
        self.goals = goals
        self.start_positions = []
        self.state_positions = []
        for i in range(1, n + 1):
            for j in range(1, m + 1):
                if (i, j) not in self.obstacles:
                    self.state_positions.append((i, j))

        self.show = True

        self._ax = None
        self.fig = None

        self.drawn_map_objects = []
        self.drawn_new_obstacles = []
        self.drawn_belief_objects = []
        self.drawn_state_object = None

        self.save_to_file = False

    # def scale(self, x_scale=2, y_scale=2):
    #     """
    #     x_scale: scaling factor for the x-direction
    #     y_scale: scaling factor for the y-direction
    #     """
    #     def int_scale(i, scale_const):
    #
    #         if scale_const > 1:
    #             if i < 1:
    #                 return i
    #             return math.ceil(i * scale_const)
    #         else:
    #             return math.floor(i * scale_const)
    #
    #     _n = int_scale(self.n, x_scale)
    #     _m = int_scale(self.m, y_scale)
    #     _obstacles = []
    #     _landmarks = []
    #     _danger_zones = []
    #     _goals = []
    #     _state_positions = []
    #
    #     for i in range(1, _n + 1):
    #         for j in range(1, _m + 1):
    #             _i, _j = int_scale(i, 1/x_scale), int_scale(j, 1/y_scale)
    #             if (_i, _j) in self.obstacles:
    #                 _obstacles.append((i, j))
    #             if (_i, _j) in self.landmarks:
    #                 _landmarks.append((i, j))
    #             if (_i, _j) in self.danger_zones:
    #                 _danger_zones.append((i, j))
    #             if (_i, _j) in self.goals:
    #                 _goals.append((i, j))
    #             if (_i, _j) in self.state_positions:
    #                 _state_positions.append((i, j))
    #
    #     return GridMap(_n, _m, _obstacles, _landmarks, _danger_zones, _goals)

    @staticmethod
    def get_plt_box(n, m, width, height, color, alpha=1.0):
        return plt.Rectangle((n, m), width, height, color=color, alpha=alpha, linewidth=0)

    @staticmethod
    def get_plt_box_with_border(n, m, width, height, color):
        return plt.Rectangle((n, m), width, height, facecolor='none', edgecolor=color, linewidth=3)

    @staticmethod
    def get_plt_circle(n, m, diameter, color):
        return plt.Circle((n + 0.5, m + 0.5), diameter/2, color=color, linewidth=0)

    def add_text_to_patch(self, patch, text, color):
        self._ax.annotate(
            text,
            (patch.get_x() + (patch.get_width() / 2), patch.get_y() + (patch.get_height() / 2)),
            color=color,
            ha='center', va='center'
        )

    def process_line(self, line_str, row, record_start=False, only_changes=False):
        def get_pos_of_char(s, ch, r):
            return [(r, i+1) for i, ltr in enumerate(s) if ltr == ch]

        if not only_changes:
            self.obstacles.extend(get_pos_of_char(line_str, '#', row))
            self.landmarks.extend(get_pos_of_char(line_str, 'L', row))
            self.danger_zones.extend(get_pos_of_char(line_str, 'D', row))
            self.goals.extend(get_pos_of_char(line_str, 'X', row))

            if record_start:
                self.start_positions.extend(get_pos_of_char(line_str, 'R', row))
        else:
            for obs in get_pos_of_char(line_str, '#', row):
                if obs not in self.obstacles:
                    self.obstacles_new.append(obs)

    def read_map_from_file(self, fp, record_start=False, only_changes=False):

        curr_line = fp.readline()  # Skip first line
        row_c = 1
        while row_c <= self.m:
            curr_line = fp.readline()
            self.process_line(curr_line, row_c, record_start, only_changes)
            row_c += 1

    def generate_map(self, record_start=False):
        self.fig, self._ax = plt.subplots()

        self._ax.set_xlim(1, self.n + 1)
        self._ax.set_ylim(1, self.m + 1)
        self._ax.set_aspect('equal', adjustable='box')
        # invert y-axis
        # plt.gca().invert_yaxis()
        self.fig.set_size_inches(10, 10)
        # self.fig.tight_layout()
        self._ax.set_xticks([])
        self._ax.set_yticks([])

        if self.show:
            plt.ion()

        self.draw_map_objects(record_start)

    def draw_map_objects(self, record_start=False):
        for obstacle in self.obstacles:
            do = self.get_plt_box(obstacle[0], obstacle[1], 1, 1, self.ObjectColors.Obstacle)
            self._ax.add_patch(do)
            self.drawn_map_objects.append(do)
        for danger in self.danger_zones:
            do = self.get_plt_box(danger[0], danger[1], 1, 1, self.ObjectColors.Danger)
            self._ax.add_patch(do)
            self.drawn_map_objects.append(do)
        for landmark in self.landmarks:
            do = self.get_plt_box(landmark[0], landmark[1], 1, 1, self.ObjectColors.Landmark)
            self._ax.add_patch(do)
            self.drawn_map_objects.append(do)
        for goal in self.goals:
            do = self.get_plt_box(goal[0], goal[1], 1, 1, self.ObjectColors.Goal)
            self._ax.add_patch(do)
            self.drawn_map_objects.append(do)
            # self.add_text_to_patch(do, 'G', self.ObjectColors.Start)
            # do.set_clip_path(do)
        if record_start:
            for sp in self.start_positions:
                do = self.get_plt_box_with_border(sp[0], sp[1], 1, 1, self.ObjectColors.Goal)
                self._ax.add_patch(do)
                self.drawn_map_objects.append(do)
                self.add_text_to_patch(do, 'S', self.ObjectColors.Start)
                do.set_clip_path(do)
        plt.pause(0.5)

    def draw_new_obstacles(self):
        for obstacle in self.obstacles_new:
            do = self.get_plt_box(obstacle[0], obstacle[1], 1, 1, self.ObjectColors.ObstacleNew)
            self._ax.add_patch(do)
            self.drawn_new_obstacles.append(do)
        plt.pause(0.5)

    def clear_new_obstacles(self):
        for plt_o in self.drawn_new_obstacles:
            plt_o.remove()
        self.drawn_new_obstacles = []
        self.obstacles_new = []

    def draw_state_and_belief(self, state, belief):
        # Remove already drawn objects
        if self.drawn_state_object is not None:
            self.drawn_state_object.remove()
        for plt_o in self.drawn_belief_objects:
            plt_o.remove()
        self.drawn_belief_objects = []

        for b in belief:
            # do = self.get_plt_circle(b[0], b[1], b[2], self.ObjectColors.Belief)
            do = self.get_plt_box(b[0], b[1], 1, 1, self.ObjectColors.Belief, b[2])
            self._ax.add_patch(do)
            self.drawn_belief_objects.append(do)

        do = self.get_plt_box(state[0], state[1], 1, 1, self.ObjectColors.Robot)
        do = self.get_plt_circle(state[0], state[1], 0.75, self.ObjectColors.Robot)
        self._ax.add_patch(do)
        self.drawn_state_object = do

        plt.pause(0.5)

    def clear_all(self):
        self.obstacles = []
        self.landmarks = []
        self.danger_zones = []
        self.goals = []

        for plt_o in self.drawn_map_objects:
            plt_o.remove()
        self.drawn_map_objects = []

        if self.drawn_state_object is not None:
            self.drawn_state_object.remove()
        self.drawn_state_object = None

        for plt_o in self.drawn_belief_objects:
            plt_o.remove()
        self.drawn_belief_objects = []

    def set_save_to_file(self, saving=False):
        self.save_to_file = saving

    @staticmethod
    def save_map_figure(folder, file_name):
        if save_to_file:
            plt.savefig(folder + f"/{file_name}.pdf")


def get_grid_map(grid_size):
    return GridMap(grid_size, grid_size, [], [], [], [])


def goto_line_start_with(fp, string):
    line = fp.readline()
    if not line:
        raise ValueError
    while not line.startswith(string):
        line = fp.readline()
        if not line:
            raise ValueError
    return line


def go_n_lines(fp, lines=1):
    ln = None
    for i in range(lines):
        ln = fp.readline()
    return ln


def parse_state_belief(state_belief_str):
    state_list = []
    re_pattern = re.compile(
        r"\(State\(\((\d+), (\d+)\) \| Goal: [TF] \| Landmark: [TF] \| Danger: [TF] \| Terminal: [TF]\), (\d.\d+)\)"
    )

    for match in re_pattern.finditer(state_belief_str):
        state_list.append((int(match.group(2)), int(match.group(1)), float(match.group(3))))

    return state_list


def parse_state(state_str):
    result = re.search(
        r"State\(\((\d+), (\d+)\) \| Goal: [TF] \| Landmark: [TF] \| Danger: [TF] \| Terminal: [TF]\)",
        state_str
    )

    return int(result.group(2)), int(result.group(1))


if __name__ == '__main__':
    print("Generating visualisation")

    log_file = (Path.cwd() / sys.argv[1]).resolve()
    output_parent_dir = (Path.cwd() / sys.argv[2]).resolve()
    trial = int(sys.argv[3])
    occ_ord_input = int(sys.argv[4])
    grid_size = int(sys.argv[5])

    save_to_file = False
    record_start = False

    regex = r"Process_(.+)_([\d]{1,2})_([MTWFS].+).out"
    rx_result = re.search(regex, log_file.name)
    output_dir_name = rx_result.group(1) + "_" + rx_result.group(2) + "_" + rx_result.group(3) + "_trial_" + str(trial) + "/"
    output_dir = output_parent_dir / output_dir_name

    if save_to_file:
        try:
            output_dir.mkdir()
        except FileExistsError:
            # Remove all the existing files
            for file in output_dir.iterdir():
                file.unlink()

    print(f"\n\nLog file: {log_file}")
    print(f"Output dir  : {output_dir}")
    print(f"Trial #     : {trial}")
    print(f"Occ. order  : {occ_ord_input}")  # Occurrence order of the trial
    print(f"Grid size   : {grid_size}x{grid_size}")

    grid_map = get_grid_map(grid_size)
    grid_map.set_save_to_file(save_to_file)

    with log_file.open('r') as fp:

        """Danger Zones"""
        # Draw deformed map
        # line = goto_line_start_with(fp, "***** PROBLEM DEFINITION *****")
        # line = go_n_lines(fp, 7)
        # grid_map.read_map_from_file(fp, record_start)
        # grid_map.generate_map(record_start)
        # grid_map.save_map_figure(str(output_dir), f"environment")

        """Obstacle"""
        # Draw original map
        line = goto_line_start_with(fp, "Original environment:")
        grid_map.read_map_from_file(fp, record_start)
        grid_map.generate_map(record_start)
        grid_map.save_map_figure(str(output_dir), f"original")
        # grid_map.clear_all()

        # # Draw deformed map
        # line = goto_line_start_with(fp, "Deformed environment:")
        # grid_map.read_map_from_file(fp, record_start=record_start, only_changes=True)
        # grid_map.draw_changes()
        # grid_map.save_map_figure(str(output_dir), f"deformed")

        occ_order = 1
        while 1:
            line = goto_line_start_with(fp, f"====== TRIAL STARTING-{trial} ======")
            if occ_order == occ_ord_input:
                break
            else:
                occ_order += 1

        step = 0

        line = goto_line_start_with(fp, "====== Initial Belief ======")

        line = fp.readline()
        # print(f"Step {step}")
        # print(f"Step {step} belief: ", line)
        belief = parse_state_belief(line)
        line = fp.readline()
        # print(f"Step {step} real: ", line)
        state = parse_state(line)
        # print(f"Step {step} belief: ", belief)
        # print(f"Step {step} real: ", state)
        grid_map.draw_state_and_belief(state, belief)
        grid_map.save_map_figure(str(output_dir), f"step_{step}")

        step += 1

        try:

            while 1:
                line = fp.readline()
                if not line:
                    print("End of File. Total Steps: ", step - 1)
                    break
                if line.startswith("====== TRIAL STARTING-"):
                    print("Trial ended. Total Steps: ", step - 1)
                    break
                if line.startswith(f"====== Step {step} ======"):
                    line = goto_line_start_with(fp, "Resulting Belief:")
                    # print(f"Step {step}")
                    # print(f"Step {step} belief: ", line)
                    belief = parse_state_belief(line)
                    line = goto_line_start_with(fp, "True State:")
                    # print(f"Step {step} real: ", line)
                    state = parse_state(line)
                    # print(f"Step {step} belief: ", belief)
                    # print(f"Step {step} real: ", state)
                    grid_map.read_map_from_file(fp, only_changes=True)
                    grid_map.draw_new_obstacles()
                    grid_map.draw_state_and_belief(state, belief)
                    grid_map.save_map_figure(str(output_dir), f"step_{step}")

                    grid_map.clear_new_obstacles()

                    step += 1

        except ValueError:
            print("File end reached")

    print("Done")
    plt.ion()

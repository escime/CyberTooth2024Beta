from commands2 import Subsystem
from wpilib import PowerDistribution, SmartDashboard


class UtilSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

        self.grid_position = [0, 0]
        self.grid_length = [5, 5]

        grid_origin = ["stow", 0.77]
        self.grid = []

        grid_column_spacing = 0
        grid_row_spacing = 1.67
        for j in range(0, self.grid_length[1]):
            grid_row = []
            for i in range(0, self.grid_length[0]):
                # grid_row.append([grid_origin[0] + (i * grid_column_spacing), grid_origin[1] + (j * grid_row_spacing)])
                grid_row.append(["stow", grid_origin[1] + (j * grid_row_spacing)])
            self.grid.append(grid_row)

        # for i in range(0, self.grid_length[1]):
        #     release = ""
        #     for j in range(0, self.grid_length[0]):
        #         release += (str(self.grid[j][i]) + " ")
        #     print(release)

    # def toggle_channel(self, on: bool) -> None:
    #     self.pdh.setSwitchableChannel(on)

    def increment_grid_position(self, x: int, y: int):
        if 0 <= self.grid_position[0] + x <= self.grid_length[0] - 1:
            self.grid_position[0] += x
        if 0 <= self.grid_position[1] + y <= self.grid_length[1] - 1:
            self.grid_position[1] += y

    def get_grid_position(self) -> [int, int]:
        return self.grid_position

    def get_grid_setpoint(self) -> [float, float]:
        return [self.grid[self.grid_position[0]][self.grid_position[1]][0],
                self.grid[self.grid_position[0]][self.grid_position[1]][1]]

    def periodic(self) -> None:
        for j in range(0, self.grid_length[1]):
            for i in range(0, self.grid_length[0]):
                displaystr = "Scoring Grid " + str(j) + "|" + str(i)
                if self.grid_position[0] == i and self.grid_position[1] == j:
                    SmartDashboard.putBoolean(displaystr, True)
                else:
                    SmartDashboard.putBoolean(displaystr, False)


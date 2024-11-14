from commands2 import Subsystem
from wpilib import PowerDistribution, SmartDashboard, DriverStation


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

        self.scoring_location = 0
        self.scoring_locations_red = [
            [13.259, 4.111, 180.001, "Red Podium"],
            [10.929, 5.364, 120, "Red Amp"],
            [10.929, 2.8, 60, "Red Source"]
        ]
        self.scoring_locations_blue = [
            [16.5 - self.scoring_locations_red[0][0], 4.111, 0.001, "Blue Podium"],
            [16.5 - self.scoring_locations_red[2][0], 2.8, 300, "Blue Source"],
            [16.5 - self.scoring_locations_red[1][0], 5.364, 240, "Blue Amp"]
        ]

        self.scoring_setpoint = 0
        self.scoring_setpoints = ["stow", "reverse_shoot", "shoot", "intake"]

    # def toggle_channel(self, on: bool) -> None:
    #     self.pdh.setSwitchableChannel(on)

    def cycle_scoring_locations(self, cycle_amount: int) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            if cycle_amount + self.scoring_location >= len(self.scoring_locations_red):
                self.scoring_location = 0
            elif cycle_amount + self.scoring_location < 0:
                self.scoring_location = len(self.scoring_locations_red) - 1
            else:
                self.scoring_location += cycle_amount
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            if cycle_amount + self.scoring_location >= len(self.scoring_locations_blue):
                self.scoring_location = 0
            elif cycle_amount + self.scoring_location < 0:
                self.scoring_location = len(self.scoring_locations_blue) - 1
            else:
                self.scoring_location += cycle_amount

    def cycle_scoring_setpoints(self, cycle_amount: int) -> None:
        if cycle_amount + self.scoring_setpoint >= len(self.scoring_setpoints):
            self.scoring_setpoint = 0
        elif cycle_amount + self.scoring_setpoint < 0:
            self.scoring_setpoint = len(self.scoring_setpoints) - 1
        else:
            self.scoring_setpoint += cycle_amount

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
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            SmartDashboard.putString("Scoring Location", self.scoring_locations_red[self.scoring_location][3])
        else:
            SmartDashboard.putString("Scoring Location", self.scoring_locations_red[self.scoring_location][3])
        SmartDashboard.putString("Scoring Setpoint", self.scoring_setpoints[self.scoring_setpoint])
        # for j in range(0, self.grid_length[1]):
        #     for i in range(0, self.grid_length[0]):
        #         displaystr = "Scoring Grid " + str(j) + "|" + str(i)
        #         if self.grid_position[0] == i and self.grid_position[1] == j:
        #             SmartDashboard.putBoolean(displaystr, True)
        #         else:
        #             SmartDashboard.putBoolean(displaystr, False)

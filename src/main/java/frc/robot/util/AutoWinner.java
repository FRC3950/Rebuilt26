package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class AutoWinner {
    /**
     * Returns an Optional containing true if we won auto, false if we lost,
     * or empty if the data is not yet available.
     * 
     * @return Optional with the result, or empty if data is unavailable
     */
    public static Optional<Boolean> getAutoWinResult() {
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData == null || gameData.isEmpty()) {
            return Optional.empty();
        }

        char autoWinnerChar = gameData.charAt(0);
        Optional<Alliance> ourAlliance = DriverStation.getAlliance();

        if (ourAlliance.isEmpty()) {
            return Optional.empty();
        }

        switch (autoWinnerChar) {
            case 'B':
                return Optional.of(ourAlliance.get() == Alliance.Blue);
            case 'R':
                return Optional.of(ourAlliance.get() == Alliance.Red);
            default:
                return Optional.empty();
        }
    }
}

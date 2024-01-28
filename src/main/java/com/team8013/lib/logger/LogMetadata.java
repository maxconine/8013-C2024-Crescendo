package com.team8013.lib.logger;

import java.util.OptionalInt;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
// Data to be logged:
// Event Name
// Match Number
// Match Type
// Alliance
// Driver Station Location
// Current Auto Path
// Disable Time

public class LogMetadata {

    private String eventName;
    private int matchNumber;
    private MatchType matchType;
    private Alliance alliance;
    private OptionalInt driverStationLocation;
    private long disableTimeUnixMillis;

    public LogMetadata(String eventName, int matchNumber, MatchType matchType, Alliance alliance,
            OptionalInt optionalInt, long disableTimeUnixMillis) {
        this.eventName = eventName;
        this.matchNumber = matchNumber;
        this.matchType = matchType;
        this.alliance = alliance;
        this.driverStationLocation = optionalInt;
        this.disableTimeUnixMillis = disableTimeUnixMillis;
    }

    public LogMetadata() {
        // I did some sus changes to this, hopefully it works still..
        this(DriverStation.getEventName(), DriverStation.getMatchNumber(), DriverStation.getMatchType(), DriverStation.getAlliance().get(), DriverStation.getLocation(), System.currentTimeMillis());
    }

    public String generateMetadataString() {
        return eventName + "\n" + matchNumber + "\n" + matchType.toString() + "\n" + alliance.toString() + "\n"
                + driverStationLocation + "\n" + disableTimeUnixMillis
                + "\n# eventName, matchNumber, matchType, alliance, driverStationLocation, disableTimeUnixMillis";

    }

}

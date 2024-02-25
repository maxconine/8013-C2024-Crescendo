package com.team8013.frc2024.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.team8013.frc2024.auto.modes.*;

public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING,
        ONE_NOTE,
        TWO_RIGHT_RED,
        TWO_MIDDLE,
        MIDDLE_AROUND,
        CW_FOUR_PIECE,
        MID_START_3_PIECE
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();

    public AutoModeSelector() {
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.setDefaultOption("One Note", DesiredMode.ONE_NOTE);
        mModeChooser.setDefaultOption("Two Right RED", DesiredMode.TWO_RIGHT_RED);
        mModeChooser.setDefaultOption("Two Middle", DesiredMode.TWO_MIDDLE);
        mModeChooser.setDefaultOption("Two Around Middle", DesiredMode.MIDDLE_AROUND);
        mModeChooser.setDefaultOption("CW FOUR PIECE", DesiredMode.CW_FOUR_PIECE);
        mModeChooser.setDefaultOption("Middle Start 3 Piece", DesiredMode.MID_START_3_PIECE);
        SmartDashboard.putData("Auto Mode", mModeChooser);
    }

    public void updateModeCreator(boolean force_regen) {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode || force_regen) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case ONE_NOTE:
                return Optional.of(new OneNote());
            case TWO_RIGHT_RED:
                return Optional.of(new TwoRightRed());
            case TWO_MIDDLE:
                return Optional.of(new TwoMiddle());
            case MIDDLE_AROUND:
                return Optional.of(new TwoAround());
            case CW_FOUR_PIECE:
                return Optional.of(new CWFourPiece());
            case MID_START_3_PIECE:
                return Optional.of(new ThreePieceMiddleStart());

            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public static SendableChooser<DesiredMode> getModeChooser() {
        return mModeChooser;
    }

    public DesiredMode getDesiredAutomode() {
        return mCachedDesiredMode;
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}

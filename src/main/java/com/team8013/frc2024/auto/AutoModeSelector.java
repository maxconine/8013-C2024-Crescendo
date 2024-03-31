package com.team8013.frc2024.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.team8013.frc2024.auto.modes.*;

public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING,
        ONE_NOTE,
        TEST_AUTO_NOTE_PICKUP,
        TWO_AMP_SIDE,
        TWO_STAGE_SIDE,
        TWO_STAGE_SIDE_RED,
        TWO_MIDDLE,
        CAUSE_CHAOS_STAGE_SIDE,
        MIDDLE_AROUND,
        CW_FOUR_PIECE,
        MID_START_3_PIECE,
        MID_START_3_PIECE_AMP_SIDE
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();

    public AutoModeSelector() {
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.setDefaultOption("One Note", DesiredMode.ONE_NOTE);
        //mModeChooser.setDefaultOption("Test Auto Note Pickup", DesiredMode.TEST_AUTO_NOTE_PICKUP);
        mModeChooser.setDefaultOption("Two Amp Side starting on side of subwoofer", DesiredMode.TWO_AMP_SIDE);
        mModeChooser.setDefaultOption("Two Stage Side starting on side of subwoofer", DesiredMode.TWO_STAGE_SIDE);
        mModeChooser.setDefaultOption("Two Stage Side RED starting on side of subwoofer", DesiredMode.TWO_STAGE_SIDE_RED);
        mModeChooser.setDefaultOption("Two Middle", DesiredMode.TWO_MIDDLE);
        mModeChooser.setDefaultOption("Cause Chaos Stage Side", DesiredMode.CAUSE_CHAOS_STAGE_SIDE);
        //mModeChooser.setDefaultOption("Two Around Middle", DesiredMode.MIDDLE_AROUND);
        //mModeChooser.setDefaultOption("CW FOUR PIECE", DesiredMode.CW_FOUR_PIECE);
        mModeChooser.setDefaultOption("Middle Start 3 Piece Stage Side", DesiredMode.MID_START_3_PIECE);
        mModeChooser.setDefaultOption("Middle Start 3 Piece Amp Side", DesiredMode.MID_START_3_PIECE_AMP_SIDE);
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
            case TEST_AUTO_NOTE_PICKUP:
                return Optional.of(new TestAutoNotePickup());
            case TWO_AMP_SIDE:
                return Optional.of(new TwoAmpSide());
            case TWO_STAGE_SIDE:
                return Optional.of(new TwoStageSide());
            case TWO_STAGE_SIDE_RED:
                return Optional.of(new TwoStageSideRed());
            case TWO_MIDDLE:
                return Optional.of(new TwoMiddle());
            case CAUSE_CHAOS_STAGE_SIDE:
                return Optional.of(new CauseChaosStageSide());
            case MIDDLE_AROUND:
                return Optional.of(new TwoAround());
            case CW_FOUR_PIECE:
                return Optional.of(new CWFourPiece());
            case MID_START_3_PIECE:
                return Optional.of(new ThreePieceMiddleStart());
            case MID_START_3_PIECE_AMP_SIDE:
                return Optional.of(new ThreePieceMiddleStartAmpSide());

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

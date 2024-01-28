package com.team8013.frc2024.shuffleboard;

import java.util.ArrayList;

import com.team8013.frc2024.shuffleboard.tabs.SwerveTab;
import com.team8013.frc2024.shuffleboard.tabs.VisionTab;

public class ShuffleBoardInteractions {

    // Trims unneccesary tabs when in competition
    public final boolean mDebug = true;

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance; 

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    private ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<ShuffleboardTabBase>();

    public FieldView mFieldView = new FieldView();
    private SwerveTab mSwerveTab = new SwerveTab();
    private VisionTab mVisionTab = new VisionTab();

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        mTabs.add(mVisionTab);
        mTabs.add(mSwerveTab);
        for(ShuffleboardTabBase tab: mTabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
        mFieldView.update();
    }
}
 
package org.firstinspires.ftc.teamcode.Systems.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    Limelight3A l;
    LLResult result = l.getLatestResult();
    public Limelight(HardwareMap h)
    {
        l = h.get(Limelight3A.class , "limelight");
        l.setPollRateHz(100);
    }
    public void periodic()
    {
        result = l.getLatestResult();
    }
    public void Start()
    {
        l.start();
    }
    public void SwitchPipelines(int p)
    {
        l.pipelineSwitch(p);
    }
    public int GetCurrentPipeline()
    {
        return result.getPipelineIndex();
    }
    public double GetTx()
    {
        return result.getTx();
    }
    public double GetTy()
    {
        return result.getTy();
    }
    public double GetTa()
    {
        return result.getTa(); //cat la suta ocupa din ecran
    }
}

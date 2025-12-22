package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

public class Datalog
{
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField shootCycleStartTime = new Datalogger.GenericField("Shoot Cycle Start Time");
    public Datalogger.GenericField shootCycleEndTime = new Datalogger.GenericField("Shoot Cycle End Time");
    public Datalogger.GenericField artifactCounter = new Datalogger.GenericField("Artifact Counter");

    public Datalog(String name)
    {
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

                // Pass through the filename
                .setFilename(name)

                // Request an automatic timestamp field
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(
                        shootCycleStartTime,
                        shootCycleEndTime,
                        artifactCounter
                )
                .build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine()
    {
        datalogger.writeLine();
    }
}

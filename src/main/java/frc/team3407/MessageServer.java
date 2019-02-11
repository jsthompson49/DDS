package frc.team3407;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MessageServer {

    private static final String ROBORIO = "RoboRio";

    private NetworkTable table;
    private NetworkTableEntry processCountEntry;
    private NetworkTableEntry messageCountEntry;
    private NetworkTableEntry messageReceiverEntry;
    private NetworkTableEntry messageSenderEntry;
    private NetworkTableEntry messageEntry;

    private long pollCount = 0;
    private long processCount;
    private long messageCount;

    private String message;
    private String sender;

    public MessageServer(String tableName) {
        NetworkTableInstance root = NetworkTableInstance.getDefault();
        table = root.getTable(tableName);

        processCountEntry = table.getEntry("processCount");
        processCount = processCountEntry.getNumber(0).longValue();

        messageCountEntry = table.getEntry("messageCount");
        messageCount = messageCountEntry.getNumber(0).longValue();

        messageReceiverEntry = table.getEntry("receiver");
        messageSenderEntry = table.getEntry("sender");
        messageEntry = table.getEntry("message");
    }

    public void pollInterval(long count) {
        if ((pollCount % count) == 0) {
            processMessages();
        }
        pollCount++;
     }

    public void processMessages() {
        processCount++;
        processCountEntry.setNumber(processCount);

        String messageReceiver = messageReceiverEntry.getString("");
        System.out.println("Processing message: receiver=" + messageReceiver);
        if (ROBORIO.equals(messageReceiver)) {
            messageCount++;
            message = messageEntry.getString("");
            sender = messageSenderEntry.getString("");
            System.out.println(String.format("%s messages, current mesage from %s: %s", messageCount, sender, message));

            messageCountEntry.setNumber(messageCount);
            messageEntry.setString("ACK");
            messageSenderEntry.setString(ROBORIO);
            messageReceiverEntry.setString(sender);
        }
    }
}

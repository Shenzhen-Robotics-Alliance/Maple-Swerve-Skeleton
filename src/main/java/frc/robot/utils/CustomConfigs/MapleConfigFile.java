// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/ and ChatGPT
package frc.robot.utils.CustomConfigs;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class MapleConfigFile {
    public final String configType;
    public final String configName;

    public static final class ConfigBlock {
        private final String blockName;
        private final Map<String, Double> doubleConfigs = new HashMap<>();
        private final Map<String, Integer> intConfigs = new HashMap<>();
        private final Map<String, String> stringConfigs = new HashMap<>();

        private final List<String> configOrders = new ArrayList<>();

        private ConfigBlock(String blockName) {
            this.blockName = blockName;
        }

        public boolean hasDoubleConfig(String name) {
            return doubleConfigs.containsKey(name);
        }

        public boolean hasIntConfig(String name) {
            return intConfigs.containsKey(name);
        }

        public boolean hasStringConfig(String name) {
            return stringConfigs.containsKey(name);
        }
        public double getDoubleConfig(String name) throws NullPointerException {
            if (!hasDoubleConfig(name))
                throw new NullPointerException(
                        "Configuration not found for block: " + blockName + ", config: " + name + ", type: double");
            return doubleConfigs.get(name);
        }

        public int getIntConfig(String name) throws NullPointerException {
            if (!hasIntConfig(name))
                throw new NullPointerException(
                        "Configuration not found for block: " + blockName + ", config: " + name + ", type: int");
            return intConfigs.get(name);
        }

        public String getStringConfig(String name) throws NullPointerException {
            if (!hasStringConfig(name))
                throw new NullPointerException(
                        "Configuration not found for block: " + blockName + ", config: " + name + ", type: string");
            return stringConfigs.get(name);
        }

        public void putDoubleConfig(String configName, double value) throws IllegalArgumentException {
            if (intConfigs.containsKey(configName) || stringConfigs.containsKey(configName))
                throw new IllegalArgumentException(
                        "Cannot put double config '"
                                + configName
                                + "' to block '"
                                + blockName
                                + "' since there is already an int or string config with the same name");
            configOrders.add(configName);
            doubleConfigs.put(configName, value);
        }

        public void putIntConfig(String configName, int value) throws IllegalArgumentException {
            if (doubleConfigs.containsKey(configName) || stringConfigs.containsKey(configName))
                throw new IllegalArgumentException(
                        "Cannot put int config '"
                                + configName
                                + "' to block '"
                                + blockName
                                + "' since there is already a double or string config with the same name");
            configOrders.add(configName);
            intConfigs.put(configName, value);
        }

        public void putStringConfig(String configName, String value) throws IllegalArgumentException {
            if (doubleConfigs.containsKey(configName) || intConfigs.containsKey(configName))
                throw new IllegalArgumentException(
                        "Cannot put string config '"
                                + configName
                                + "' to block '"
                                + blockName
                                + "' since there is already a double or int config with the same name");
            configOrders.add(configName);
            stringConfigs.put(configName, value);
        }

        public String getBlockName() {
            return this.blockName;
        }
    }

    private final Map<String, ConfigBlock> configBlocks = new HashMap<>();
    private final List<String> configBlocksOrder = new ArrayList<>();

    public MapleConfigFile(String configType, String configName) {
        this.configType = configType;
        this.configName = configName;
    }

    public ConfigBlock getBlock(String blockName) {
        if (!configBlocksOrder.contains(blockName)) {
            configBlocks.put(blockName, new ConfigBlock(blockName));
            configBlocksOrder.add(blockName);
        }
        return configBlocks.get(blockName);
    }

    public static MapleConfigFile fromDeployedConfig(String configType, String configName)
            throws IllegalArgumentException, IOException {
        MapleConfigFile configFile = new MapleConfigFile(configType, configName);

        final Path xmlFilePath =
                Paths.get(
                        Filesystem.getDeployDirectory().getPath(), "configs", configType, configName + ".xml");
        File xmlFile = xmlFilePath.toFile();
        if (!xmlFile.exists()) {
            throw new IOException("Config file does not exist: " + xmlFilePath);
        }

        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(xmlFile);
            doc.getDocumentElement().normalize();

            if (!doc.getDocumentElement().getNodeName().equals(configType)) {
                throw new IllegalArgumentException("Root element is not " + configType);
            }

            NodeList blocks = doc.getDocumentElement().getChildNodes();
            processBlocks(configFile, blocks);
        } catch (Exception e) {
            throw new IOException("Error reading config file", e);
        }

        return configFile;
    }

    private static void processBlocks(MapleConfigFile configFile, NodeList blocks) {
        for (int i = 0; i < blocks.getLength(); i++) {
            Node blockNode = blocks.item(i);
            if (blockNode.getNodeType() == Node.ELEMENT_NODE) {
                Element blockElement = (Element) blockNode;
                String blockName = blockElement.getTagName();
                ConfigBlock block = new ConfigBlock(blockName);
                readBlockConfig(blockElement, block);
                configFile.configBlocks.put(blockName, block);
                configFile.configBlocksOrder.add(blockName);
            }
        }
    }

    private static void readBlockConfig(Element blockElement, ConfigBlock block) {
        NodeList configNodes = blockElement.getChildNodes();
        for (int j = 0; j < configNodes.getLength(); j++) {
            Node configNode = configNodes.item(j);
            if (configNode.getNodeType() == Node.ELEMENT_NODE) {
                addConfigToBlock((Element) configNode, block);
            }
        }
    }

    private static void addConfigToBlock(Element configElement, ConfigBlock block) {
        String configTag = configElement.getTagName();
        String type = configElement.getAttribute("type");
        String value = configElement.getTextContent();

        switch (type) {
            case "double":
                block.putDoubleConfig(configTag, Double.parseDouble(value));
                break;
            case "int":
                block.putIntConfig(configTag, Integer.parseInt(value));
                break;
            case "string":
                block.putStringConfig(configTag, value);
                break;
        }
    }

    public void saveConfigToUSBSafe() {
        try {
            saveConfigToUSB();
        } catch (IOException ignored) {
        }
    }

    public void saveConfigToUSB() throws IOException {
        File usbDir = new File("/U/");
        if (!usbDir.exists()) {
            throw new IOException("No USB connected");
        }
        File configDir = new File(usbDir, "savedConfigs/" + this.configType);
        if (!configDir.exists() && !configDir.mkdirs()) {
            throw new IOException("Failed to create config directory on USB");
        }
        File configFile = new File(configDir, this.configName + ".xml");
        try (FileWriter writer = new FileWriter(configFile)) {
            writer.write("<" + this.configType + ">\n");
            writeAllConfigBlocks(this, writer);
            writer.write("</" + this.configType + ">\n");
        }
    }

    private static void writeAllConfigBlocks(MapleConfigFile config, FileWriter writer) throws IOException {
        for (String blockName : config.configBlocksOrder) {
            ConfigBlock block = config.configBlocks.get(blockName);
            writer.write("    <" + block.blockName + ">\n");
            writeSingleConfigBlock(block, writer);
            writer.write("    </" + block.blockName + ">\n");
        }
    }

    private static void writeSingleConfigBlock(ConfigBlock block, FileWriter writer) throws IOException {
        for (String configName:block.configOrders) {
            if (block.hasStringConfig(configName))
                writeStringConfig(block, configName, writer);
            else if (block.hasIntConfig(configName))
                writeIntConfig(block, configName, writer);
            else if (block.hasDoubleConfig(configName))
                writeDoubleConfig(block, configName, writer);
        }
    }

    private static void writeDoubleConfig(ConfigBlock block, String configName, FileWriter writer) throws IOException {
        writer.write("        <"
                + configName
                + " type=\"double\">"
                + block.getDoubleConfig(configName)
                + "</"
                + configName
                + ">\n"
        );
    }

    private static void writeStringConfig(ConfigBlock block, String configName, FileWriter writer) throws IOException {
        writer.write("        <"
                + configName
                + " type=\"string\">"
                + block.getStringConfig(configName)
                + "</"
                + configName
                + ">\n"
        );
    }

    private static void writeIntConfig(ConfigBlock block, String configName, FileWriter writer) throws IOException {
        writer.write("        <"
                + configName
                + " type=\"int\">"
                + block.getIntConfig(configName)
                + "</"
                + configName
                + ">\n"
        );
    }
}

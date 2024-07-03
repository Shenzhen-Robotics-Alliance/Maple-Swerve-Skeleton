package frc.robot.utils;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

public class RobotConfigReader {

    /**
     * the hashmap that stores all the configs
     */
    private Map<String, Map<String, Double>> robotConfigs = new HashMap(1);

    /**
     * the configurations to tune, in the form of configDomain/configName
     */
    private final List<String> configsToTune = new ArrayList(1);

    private File xmlFile;
    private Document doc;
    XPathFactory xPathFactory;
    XPath xPath;

    public RobotConfigReader() {
        try {
            readConfigs("robotConfig");
            // System.out.println("robot config: " + robotConfigs);
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("error while reading robot config:" + e);
        }
    }

    public RobotConfigReader(String configName) {
        try {
            readConfigs("robotConfig");
            readConfigs(configName);
            // System.out.println("robot config: " + robotConfigs);
        } catch (Exception e) {
            throw new RuntimeException("error while reading robot config:" + e);
        }
    }

    private void readConfigs(String configName)
            throws IOException, SAXException, ParserConfigurationException, XPathExpressionException {
        /* read xml file from filesystem */
        xmlFile = new File(Filesystem.getDeployDirectory(), "configs/" + configName + ".xml");
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        doc = dBuilder.parse(xmlFile);
        doc.getDocumentElement().normalize();

        /* add xpath finder */
        xPathFactory = XPathFactory.newInstance();
        xPath = xPathFactory.newXPath();

        /* Get all configuration elements (e.g., hardwareConfig, chassisConfig) */
        XPathExpression expr = xPath.compile("/robotConfig/*");
        NodeList configDomains = (NodeList) expr.evaluate(doc, XPathConstants.NODESET);

        for (int i = 0; i < configDomains.getLength(); i++) {
            Node configNode = configDomains.item(i);
            String domainName = configNode.getNodeName();
            readDomain(domainName);
        }
    }

    private void readDomain(String domainName) throws XPathExpressionException {
        /* read the configurations for hardware */
        XPathExpression expr = xPath.compile("/robotConfig/" + domainName + "/*");
        NodeList nodes = (NodeList) expr.evaluate(doc, XPathConstants.NODESET);

        Map<String, Double> domainConfigs =
                robotConfigs.containsKey(domainName)
                        ? // keep original configs if there already is
                        robotConfigs.get(domainName)
                        : new HashMap();
        for (int i = 0; i < nodes.getLength(); i++) {
            String constantName = nodes.item(i).getNodeName();
            readConstant(domainName, constantName, domainConfigs);
        }

        robotConfigs.put(domainName, domainConfigs);
    }

    /**
     * read a specific constant from the xml file
     *
     * @param domainName    the name of the domain that the constant belongs to
     * @param constantName  the name of that constant
     * @param domainConfigs the current map of the configurations inside the domain that this constant
     *                      belongs to
     */
    private void readConstant(
            String domainName, String constantName, Map<String, Double> domainConfigs)
            throws XPathExpressionException {
        /* only reads double and int, for boolean, just do int and then do param != 0 to judge true or false */

        // XPathExpression expr = xPath.compile("/robotConfig/hardware/" + constantName + "/text()");
        XPathExpression expr = xPath.compile("/robotConfig/" + domainName + "/" + constantName);
        Node node = (Node) expr.evaluate(doc, XPathConstants.NODE);

        if (node == null) {
            System.out.println("warning, constant: " + constantName + " not found in xml file, skipping");
            return;
        }

        domainConfigs.put(constantName, Double.parseDouble(node.getTextContent()));
        // System.out.println("reading " + domainName + " constant: " + constantName + ", value: " +
        // node.getTextContent());
    }

    /**
     * gets the configuration in a given path
     *
     * @param configPath in domainName/constantName
     * @return the value of the constant
     */
    public double getConfig(String configPath) {
        try {
            String domainName = configPath.split("/")[0];
            String constantName = configPath.split("/")[1];
            return getConfig(domainName, constantName);
        } catch (ArrayIndexOutOfBoundsException e) {
            throw new IllegalArgumentException("invalid path: " + configPath);
        }
    }

    /**
     * gets the configuration in a given path
     *
     * @param domainName   the name of the domain that the constant belongs to
     * @param constantName the name of the constant
     * @return the value of the constant
     */
    public double getConfig(String domainName, String constantName) {
        try {
            return robotConfigs.get(domainName).get(constantName);
        } catch (NullPointerException e) {
            throw new NullPointerException("config not found: " + domainName + "/" + constantName);
        }
    }

    /**
     * start to tune a configuration on the dashboard (shuffleboard suggested)
     */
    public void startTuningConfig(String domainName, String constantName) {
        startTuningConfig(domainName + "/" + constantName);
    }

    public void startTuningConfig(String configPath) {
        if (!configsToTune.contains(configPath)) configsToTune.add(configPath);
        SmartDashboard.putNumber(configPath, getConfig(configPath));
    }

    public void updateTuningConfigsFromDashboard() {
        for (String configToTune : configsToTune) {
            String domainName = configToTune.split("/")[0];
            String constantName = configToTune.split("/")[1];
            Map<String, Double> domainConfig = robotConfigs.get(domainName);
            domainConfig.put(
                    constantName, SmartDashboard.getNumber(configToTune, domainConfig.get(constantName)));
            robotConfigs.put(domainName, domainConfig);
        }
    }
}

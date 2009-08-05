package edu.umich.soar.gridmap2d.soar;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import edu.umich.soar.gridmap2d.CognitiveArchitecture;
import edu.umich.soar.gridmap2d.Game;
import edu.umich.soar.gridmap2d.Gridmap2D;
import edu.umich.soar.gridmap2d.Names;
import edu.umich.soar.gridmap2d.config.ClientConfig;
import edu.umich.soar.gridmap2d.config.SoarConfig;
import edu.umich.soar.gridmap2d.players.Eater;
import edu.umich.soar.gridmap2d.players.EaterCommander;
import edu.umich.soar.gridmap2d.players.RobotCommander;
import edu.umich.soar.gridmap2d.players.Robot;
import edu.umich.soar.gridmap2d.players.Tank;
import edu.umich.soar.gridmap2d.players.TankCommander;
import edu.umich.soar.gridmap2d.players.Taxi;
import edu.umich.soar.gridmap2d.players.TaxiCommander;
import edu.umich.soar.gridmap2d.world.RoomWorld;

import sml.Agent;
import sml.ConnectionInfo;
import sml.Kernel;
import sml.smlPrintEventId;
import sml.smlRunStepSize;
import sml.smlSystemEventId;
import sml.smlUpdateEventId;
import sml.sml_Names;
import sml.smlRunFlags;

public class Soar implements CognitiveArchitecture, Kernel.UpdateEventInterface, Kernel.SystemEventInterface {

	private static Logger logger = Logger.getLogger(Soar.class);

	private boolean runTilOutput = false;
	private Kernel kernel = null;
	
	private class AgentData {
		AgentData(Agent agent, File productions) {
			this.agent = agent;
			this.productions = productions;
		}
		
		Agent agent;
		File productions;
	}
	
	private Map<String, AgentData> agents = new HashMap<String, AgentData>();
	private String basePath;
	private Map<String, ClientConfig> clients;
	private int maxMemoryUsage;
	private boolean soarPrint;
	private int port;
	private boolean debug;
	
	/**
	 * @param config
	 * @param clients
	 * @param game
	 * @param basePath
	 * 
	 * @throws IllegalStateException If there is an unrecoverable error initializing Soar
	 */
	public Soar(SoarConfig config, Map<String, ClientConfig> clients, Game game, String basePath) {
		this.basePath = basePath;
		this.runTilOutput = config.runTilOutput(game);
		
		this.clients = clients;
		this.maxMemoryUsage = config.max_memory_usage;
		this.soarPrint = config.soar_print;
		this.port = config.port;
		this.debug = config.spawn_debuggers;
		
		if (config.remote != null) {
			kernel = Kernel.CreateRemoteConnection(true, config.remote, config.port);
		} else {
			// Create kernel
			kernel = Kernel.CreateKernelInNewThread("SoarKernelSML", config.port);
			//kernel = Kernel.CreateKernelInCurrentThread("SoarKernelSML", true);
		}

		if (kernel.HadError()) {
			throw new IllegalStateException(Names.Errors.kernelCreation + kernel.GetLastErrorDescription());
		}
		
		// We want the most performance
		logger.debug(Names.Debug.autoCommit);
		kernel.SetAutoCommit(false);

		// Register for events
		logger.trace(Names.Trace.eventRegistration);
		kernel.RegisterForSystemEvent(smlSystemEventId.smlEVENT_SYSTEM_START, this, null);
		kernel.RegisterForSystemEvent(smlSystemEventId.smlEVENT_SYSTEM_STOP, this, null);
		
		if (runTilOutput) {
			logger.debug(Names.Debug.runTilOutput);
			kernel.RegisterForUpdateEvent(smlUpdateEventId.smlEVENT_AFTER_ALL_GENERATED_OUTPUT, this, null);
		} else {
			logger.debug(Names.Debug.noRunTilOutput);
			kernel.RegisterForUpdateEvent(smlUpdateEventId.smlEVENT_AFTER_ALL_OUTPUT_PHASES, this, null);
		}
		
	}
	
	public boolean debug() {
		return debug;
	}
	
	public void seed(int seed) {
		kernel.ExecuteCommandLine("srand " + seed, null) ;
	}
	
	public void doBeforeClients() {
		// Start or wait for clients (false: before agent creation)
		logger.trace(Names.Trace.beforeClients);
		doClients(false);
		
	}
	
	public void doAfterClients() {
		// Start or wait for clients (true: after agent creation)
		logger.trace(Names.Trace.afterClients);
		doClients(true);
		
	}
	
	private void doClients(boolean after) {
		for (Entry<String, ClientConfig> entry : clients.entrySet()) {
			if (entry.getValue().after != after) {
				continue;
			}
			
			if (entry.getKey().equals(Names.kDebuggerClient)) {
				continue;
			}
			
			if (entry.getValue().command != null) {
				spawnClient(entry.getKey(), entry.getValue());
			} else {
				if (!waitForClient(entry.getKey(), entry.getValue())) {
					Gridmap2D.control.errorPopUp(Names.Errors.clientSpawn + entry.getKey());
					return;
				}
			}
		}
	}

	private class Redirector extends Thread {
		BufferedReader br;
		public Redirector(BufferedReader br) {
			this.br = br;
		}
		
		public void run() {
			String line;
			try {
				while ((line = br.readLine()) != null) {
					System.out.println(line);
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
	
	public void spawnClient(String clientID, ClientConfig clientConfig) {
		Runtime r = java.lang.Runtime.getRuntime();
		logger.trace(Names.Trace.spawningClient + clientID);

		try {
			Process p = r.exec(clientConfig.command);
			
			InputStream is = p.getInputStream();
			InputStreamReader isr = new InputStreamReader(is);
			BufferedReader br = new BufferedReader(isr);
			Redirector rd = new Redirector(br);
			rd.start();

			is = p.getErrorStream();
			isr = new InputStreamReader(is);
			br = new BufferedReader(isr);
			rd = new Redirector(br);
			rd.start();
			
			if (!waitForClient(clientID, clientConfig)) {
				Gridmap2D.control.errorPopUp(Names.Errors.clientSpawn + clientID);
				return;
			}
			
		} catch (IOException e) {
			e.printStackTrace();
			Gridmap2D.control.errorPopUp("IOException spawning client: " + clientID);
			return;
		}
	}
	
	/**
	 * @param client the client to wait for
	 * @return true if the client connected within the timeout
	 * 
	 * waits for a client to report ready
	 */
	public boolean waitForClient(String clientID, ClientConfig clientConfig) {
		boolean ready = false;
		// do this loop if timeout seconds is 0 (code for wait indefinitely) or if we have tries left
		for (int tries = 0; (clientConfig.timeout == 0) || (tries < clientConfig.timeout); ++tries) {
			kernel.GetAllConnectionInfo();
			if (kernel.HasConnectionInfoChanged()) {
				for (int i = 0; i < kernel.GetNumberConnections(); ++i) {
					ConnectionInfo info =  kernel.GetConnectionInfo(i);
					if (info.GetName().equalsIgnoreCase(clientID)) {
						if (info.GetAgentStatus().equalsIgnoreCase(sml_Names.getKStatusReady())) {
							ready = true;
							break;
						}
					}
				}
				if (ready) {
					break;
				}
			}
			try { 
				logger.trace(Names.Trace.waitClient + clientID);
				Thread.sleep(1000); 
			} catch (InterruptedException ignored) {}
		}
		return ready;
	}
	
	public void runForever() {
		if (runTilOutput) {
			kernel.RunAllAgentsForever(smlRunStepSize.sml_UNTIL_OUTPUT);
		} else {
			kernel.RunAllAgentsForever();
		}
		
	}

	public void runStep() {
		if (runTilOutput) {
			kernel.RunAllTilOutput(smlRunStepSize.sml_UNTIL_OUTPUT);
		} else {
			kernel.RunAllAgents(1);
		}
	}

	public void destroyPlayer(String name) {
		// get the agent (human agents return null here)
		AgentData agentData = agents.remove(name);
		if (agentData == null) {
			return;
		}
		// there was an agent, destroy it
		kernel.DestroyAgent(agentData.agent);
		agentData.agent.delete();
	}
	
	public void shutdown() {
		if (kernel != null) {
			logger.trace(Names.Trace.kernelShutdown);
			kernel.Shutdown();
			kernel.delete();
		}

	}

	public String getAgentPath() {
		return basePath + "agents" + System.getProperty("file.separator");
	}
	
	/**
	 * Logger for Kernel print events
	 * @author Scott Lathrop
	 *
	 */
	private PrintLogger getLogger() { return PrintLogger.getLogger(); }
	
	
	private static class PrintLogger implements Agent.PrintEventInterface
	{
		protected static PrintLogger m_Logger = null;
		
		public static PrintLogger getLogger() 
		{
			if (m_Logger == null) {
				m_Logger = new PrintLogger();
			}
			
			return m_Logger;
		}
		
		/**
		 * @brief - callback from SoarKernel for print events
		 */
		public void printEventHandler (int eventID, Object data, Agent agent, String message) 
		{
			if (eventID == smlPrintEventId.smlEVENT_PRINT.swigValue()) {
				logger.info(message);
			}
				
		} // SoarAgentprintEventHandler	
		
		private PrintLogger () {}
		
	} // Logger
	
	private Agent createSoarAgent(String name, String productions, boolean debug) {
		Agent agent = kernel.CreateAgent(name);
		if (agent == null) {
			Gridmap2D.control.errorPopUp("Error creating agent " + name + ", " + kernel.GetLastErrorDescription());
			return null;
		}
		
		// now load the productions
		File productionsFile = new File(productions);
		if (!agent.LoadProductions(productionsFile.getAbsolutePath())) {
			Gridmap2D.control.errorPopUp("Error loading productions " + productionsFile + " for " + name + ", " + agent.GetLastErrorDescription());
			return null;
		}
		
		// if requested, set max memory usage
		int maxmem = maxMemoryUsage;
		if (maxmem > 0) {
			agent.ExecuteCommandLine("max-memory-usage " + Integer.toString(maxmem));
		}
		
		// Scott Lathrop --  register for print events
		if (soarPrint) {
			agent.RegisterForPrintEvent(smlPrintEventId.smlEVENT_PRINT, getLogger(), null,true);
		}
		
		// save the agent
		agents.put(name, new AgentData(agent, productionsFile));
		
		// spawn the debugger if we're supposed to
		if (debug && !isClientConnected(Names.kDebuggerClient)) {
			ClientConfig debuggerConfig = clients.get(Names.kDebuggerClient);
			debuggerConfig.command = getDebuggerCommand(name);

			spawnClient(Names.kDebuggerClient, debuggerConfig);
		}
		
		return agent;
	}

	/**
	 * @param client the client in question
	 * @return true if it is connected
	 * 
	 * check to see if the client specified by the client config is connected or not
	 */
	public boolean isClientConnected(String clientId) {
		boolean connected = false;
		kernel.GetAllConnectionInfo();
		for (int i = 0; i < kernel.GetNumberConnections(); ++i) {
			ConnectionInfo info =  kernel.GetConnectionInfo(i);
			if (info.GetName().equalsIgnoreCase(clientId)) {
				connected = true;
				break;
			}
		}
		return connected;
	}
	
	/**
	 * @param agentName tailor the command to this agent name
	 * @return a string command line to execute to spawn the debugger
	 */
	public String getDebuggerCommand(String agentName) {
		// Figure out whether to use java or javaw
		String os = System.getProperty("os.name");
		String commandLine;
		if (os.matches(".+indows.*") || os.matches("INDOWS")) {
			commandLine = "javaw -jar \"" + basePath 
			+ "..\\..\\SoarLibrary\\bin\\SoarJavaDebugger.jar\" -cascade -remote -agent " 
			+ agentName + " -port " + port;
		} else {
			commandLine = System.getProperty("java.home") + "/bin/java -jar " + basePath
			+ "../../SoarLibrary/bin/SoarJavaDebugger.jar -XstartOnFirstThread -cascade -remote -agent " 
			+ agentName + " -port " + port;
		}
		
		return commandLine;
	}

	public void reload(String player) {
		AgentData agentData = agents.get(player);
		if (agentData == null) {
			logger.warn("Didn't find player to reload: " + player);
			return;
		}
		
		agentData.agent.LoadProductions(agentData.productions.getAbsolutePath());
	}

	public boolean haveAgents() {
		return agents.size() > 0;
	}

	public EaterCommander createEaterCommander(Eater eater, String productions,
			int vision, String[] shutdownCommands, File mapMetadataFile, boolean debug) {
		Agent agent = createSoarAgent(eater.getName(), productions, debug);
		if (agent == null) {
			return null;
		}
		return new SoarEater(eater, agent, vision, shutdownCommands);
	}

	public TankCommander createTankCommander(Tank tank, String productions,
			String[] shutdownCommands, File mapMetadataFile, boolean debug) {
		Agent agent = createSoarAgent(tank.getName(), productions, debug);
		if (agent == null) {
			return null;
		}
		return new SoarTank(tank, agent, shutdownCommands);
	}

	public TaxiCommander createTaxiCommander(Taxi taxi, String productions,
			String[] shutdownCommands, File mapMetadataFile, boolean debug) {
		Agent agent = createSoarAgent(taxi.getName(), productions, debug);
		if (agent == null) {
			return null;
		}
		return new SoarTaxi(taxi, agent, shutdownCommands);
	}
	   
	@Override
	public RobotCommander createRoomCommander(Robot player, RoomWorld world, String productions,
			String[] shutdownCommands, File metadataFile, boolean debug) {
		Agent agent = createSoarAgent(player.getName(), productions, debug);
		if (agent == null) {
			return null;
		}
		return new SoarRobot(player, agent, kernel, world, shutdownCommands);
	}

  	public void updateEventHandler(int eventID, Object data, Kernel kernel, int runFlags) {

  		// check for override
  		int dontUpdate = runFlags & smlRunFlags.sml_DONT_UPDATE_WORLD.swigValue();
  		if (dontUpdate != 0) {
  			logger.warn(Names.Warn.noUpdate);
  			return;
  		}
  		
  		// this updates the world
  		Gridmap2D.control.tickEvent();
  		
		// Test this after the world has been updated, in case it's asking us to stop
		if (Gridmap2D.control.isStopped()) {
			// the world has asked us to kindly stop running
  			logger.debug(Names.Debug.stopRequested);
  			
  			// note that soar actually controls when we stop
  			kernel.StopAllAgents();
  		}
  	}
  	
   public void systemEventHandler(int eventID, Object data, Kernel kernel) {
  		if (eventID == smlSystemEventId.smlEVENT_SYSTEM_START.swigValue()) {
  			// soar says go
  			Gridmap2D.control.startEvent();
  		} else if (eventID == smlSystemEventId.smlEVENT_SYSTEM_STOP.swigValue()) {
  			// soar says stop
  			Gridmap2D.control.stopEvent();
  		} else {
  			// soar says something we weren't expecting
  			logger.warn(Names.Warn.unknownEvent + eventID);
 		}
   }

}
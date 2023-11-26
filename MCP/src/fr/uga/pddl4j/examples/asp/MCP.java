package fr.uga.pddl4j.examples.asp;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerConfiguration;
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
import fr.uga.pddl4j.planners.SearchStrategy;
import fr.uga.pddl4j.planners.statespace.search.StateSpaceSearch;
import fr.uga.pddl4j.planners.Statistics;
import fr.uga.pddl4j.planners.statespace.HSP;
import fr.uga.pddl4j.problem.*;
import fr.uga.pddl4j.planners.InvalidConfigurationException;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;

import java.io.*;
import java.util.*;


/**
 * Monte Carlo tree searching planning with pure random walk.
 *
 * @author Thomas FORET and Mohamed Taha MAATA
 */
@CommandLine.Command(name = "MCP",
version = "MCP 1.0",
description = "Solves a specified planning problem using Monte Carlo search strategy.",
sortOptions = false,
mixinStandardHelpOptions = true,
headerHeading = "Usage:%n",
synopsisHeading = "%n",
descriptionHeading = "%nDescription:%n%n",
parameterListHeading = "%nParameters:%n",
optionListHeading = "%nOptions:%n")
public class MCP extends AbstractPlanner {
	/**
	 * The class logger.
	 */
	private static final Logger LOGGER = LogManager.getLogger(MCP.class.getName());

    /**
     * The HEURISTIC property used for planner configuration.
     */
    public static final String HEURISTIC_SETTING = "HEURISTIC";

    /**
     * The default value of the HEURISTIC property used for planner configuration.
     */
    public static final StateHeuristic.Name DEFAULT_HEURISTIC = StateHeuristic.Name.FAST_FORWARD;

    /**
     * The WEIGHT_HEURISTIC property used for planner configuration.
     */
    public static final String WEIGHT_HEURISTIC_SETTING = "WEIGHT_HEURISTIC";

    /**
     * The default value of the WEIGHT_HEURISTIC property used for planner configuration.
     */
    public static final double DEFAULT_WEIGHT_HEURISTIC = 1.0;
	/**
	 * The weight of the heuristic.
	 */
	private double heuristicWeight = 1;

	/**
	 * The name of the heuristic used by the planner.
	 */
	private StateHeuristic.Name heuristic;

	/**
	 * The number of random walks.
	 */
	public static long NUM_WALK = 2000;

	/**
	 * The length of a random walk.
	 */
	public static long LENGTH_WALK = 10;

	/**
	 * The number of steps before restarting the search.
	 */
	public static long MAX_STEPS = 7;

    /**
     * Creates a new Monte-Carlo search planner with the default configuration.
     */
    public MCP() {
        this(MCP.getDefaultConfiguration());
    }

    /**
     * Creates a new Monte-Carlo search planner with a specified configuration.
     *
     * @param configuration the configuration of the planner.
     */
    public MCP(final PlannerConfiguration configuration) {
        super();
        this.setConfiguration(configuration);
    }

    /**
     * Sets the weight of the heuristic.
     *
     * @param weight the weight of the heuristic. The weight must be greater than 0.
     * @throws IllegalArgumentException if the weight is strictly less than 0.
     */
    @CommandLine.Option(names = {"-w", "--weight"}, defaultValue = "1.0",
        paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        if (weight <= 0) {
            throw new IllegalArgumentException("Weight <= 0");
        }
        this.heuristicWeight = weight;
    }

    /**
     * Set the name of heuristic used by the planner to the solve a planning problem.
     *
     * @param heuristic the name of the heuristic.
     */
    @CommandLine.Option(names = {"-e", "--heuristic"}, defaultValue = "FAST_FORWARD",
        description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
            + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }

	/**
	 * Returns the name of the heuristic used by the planner to solve a planning problem.
	 *
	 * @return the name of the heuristic used by the planner to solve a planning problem.
	 */
	public final StateHeuristic.Name getHeuristic() {
		return this.heuristic;
	}

	/**
	 * Returns the weight of the heuristic.
	 *
	 * @return the weight of the heuristic.
	 */
	public final double getHeuristicWeight() {
		return this.heuristicWeight;
	}

	/**
	 * Instantiates the planning problem from a parsed problem.
	 *
	 * @param problem the problem to instantiate.
	 * @return the instantiated planning problem or null if the problem cannot be instantiated.
	 */
	@Override
	public Problem instantiate(DefaultParsedProblem problem) {
		final Problem pb = new DefaultProblem(problem);
		pb.instantiate();
		return pb;
	}

	/**
	 * Search a solution plan to a specified domain and problem using MCP.
	 *
	 * @param problem the problem to solve.
	 * @return the plan found or null if no plan was found.
	 */
	@Override
	public Plan solve(Problem problem) {
		LOGGER.info("* Starting Monte-Carlo search\n");
		final long begin = System.currentTimeMillis();
		final Plan plan = this.monteCarlo(problem);
		final long end = System.currentTimeMillis();
		if (plan != null) {
			LOGGER.info("* Monte-Carlo search succeeded\n");
			this.getStatistics().setTimeToSearch(end - begin);			} else {
			LOGGER.info("* Monte-Carlo search failed\n");
		}
		return plan;
	}
	
 /**
     * Checks the planner configuration and returns if the configuration is valid.
     * A configuration is valid if (1) the domain and the problem files exist and
     * can be read, (2) the timeout is greater than 0, (3) the weight of the
     * heuristic is greater than 0 and (4) the heuristic is a not null.
     *
     * @return <code>true</code> if the configuration is valid <code>false</code> otherwise.
     */
    public boolean hasValidConfiguration() {
        return super.hasValidConfiguration()
            && this.getHeuristicWeight() > 0.0
            && this.getHeuristic() != null;
    }
	
    /**
     * This method return the default arguments of the planner.
     *
     * @return the default arguments of the planner.
     * @see PlannerConfiguration
     */
    public static PlannerConfiguration getDefaultConfiguration() {
        PlannerConfiguration config = Planner.getDefaultConfiguration();
        config.setProperty(MCP.HEURISTIC_SETTING, MCP.DEFAULT_HEURISTIC.toString());
        config.setProperty(MCP.WEIGHT_HEURISTIC_SETTING,
            Double.toString(MCP.DEFAULT_WEIGHT_HEURISTIC));
        return config;
    }

    /**
     * Returns the configuration of the planner.
     *
     * @return the configuration of the planner.
     */
    @Override
    public PlannerConfiguration getConfiguration() {
        final PlannerConfiguration config = super.getConfiguration();
        config.setProperty(MCP.HEURISTIC_SETTING, this.getHeuristic().toString());
        config.setProperty(MCP.WEIGHT_HEURISTIC_SETTING, Double.toString(this.getHeuristicWeight()));
        return config;
    }

    /**
     * Sets the configuration of the planner. If a planner setting is not defined in
     * the specified configuration, the setting is initialized with its default value.
     *
     * @param configuration the configuration to set.
     */
    @Override
    public void setConfiguration(final PlannerConfiguration configuration) {
        super.setConfiguration(configuration);
        if (configuration.getProperty(MCP.WEIGHT_HEURISTIC_SETTING) == null) {
            this.setHeuristicWeight(MCP.DEFAULT_WEIGHT_HEURISTIC);
        } else {
            this.setHeuristicWeight(Double.parseDouble(configuration.getProperty(
                MCP.WEIGHT_HEURISTIC_SETTING)));
        }
        if (configuration.getProperty(MCP.HEURISTIC_SETTING) == null) {
            this.setHeuristic(MCP.DEFAULT_HEURISTIC);
        } else {
            this.setHeuristic(StateHeuristic.Name.valueOf(configuration.getProperty(
                MCP.HEURISTIC_SETTING)));
        }
    }

	/**
	 * The main method of the <code>MCP</code> planner.
	 * Launch both MCP and HSP on all the problems of the blocks, depot, gripper and logistics domains.
	 * Write the results in a csv file.
	 *
	 * @param args the arguments of the command line.
	 */
	public static void main(String[] args) throws IOException {
		try {
			final MCP mcpPlanner = new MCP();
			final HSP hspPlanner = new HSP();
			File resultFile = new File("src/pddl/data.csv");
			BufferedWriter writer = new BufferedWriter(new FileWriter(resultFile));
			writer.write("domain,problem_number,MCP_time,MCP_length,HSP_time,HSP_length");
			writer.newLine();
			List<File> blocks_pb = List.of(new File("src/pddl/blocks").listFiles());
			List<File> depot_pb = List.of(new File("src/pddl/depot").listFiles());
			List<File> gripper_pb = List.of(new File("src/pddl/gripper").listFiles());
			List<File> logistics_pb = List.of(new File("src/pddl/logistics").listFiles());
			Map<File, List<File>> pddlFiles = new TreeMap<>();
			pddlFiles.put(new File("src/pddl/blocks_domain.pddl"), blocks_pb);
			pddlFiles.put(new File("src/pddl/depot_domain.pddl"), depot_pb);
			pddlFiles.put(new File("src/pddl/gripper_domain.pddl"), gripper_pb);
			pddlFiles.put(new File("src/pddl/logistics_domain.pddl"), logistics_pb);
			for(File domainFile : pddlFiles.keySet()) {
				for(File problemFile : pddlFiles.get(domainFile)) {
					String domainPath = domainFile.getPath();
					String problemPath = problemFile.getPath();
					mcpPlanner.setDomain(domainPath);
					hspPlanner.setDomain(domainPath);
					mcpPlanner.setProblem(problemPath);
					hspPlanner.setProblem(problemPath);
					String mcpResults = launch(mcpPlanner);
					String hspResults = launch(hspPlanner);
					String domain = domainFile.getName();
					domain = domain.substring(0, domainFile.getName().lastIndexOf("_"));
					String problem = String.valueOf(pddlFiles.get(domainFile).indexOf(problemFile) + 1);
					writer.write(domain + "," + problem + "," + mcpResults + "," + hspResults);
					writer.newLine();
				}
			}
			writer.close();
		} catch (IllegalArgumentException e) {
			LOGGER.fatal(e.getMessage());
		}
	}

	/**
	 * Launch a planner on a problem.
	 * @param planner the planner
	 * @return the results of the planner
	 * @throws FileNotFoundException
	 */
	private static String launch(AbstractPlanner planner) throws FileNotFoundException {
		try {
			Plan p = planner.solve();
			Statistics s = planner.getStatistics();
			double TimeSpent = s.getTimeToParse() + s.getTimeToEncode() + s.getTimeToSearch();
			int planLength = p == null ? 0 : p.size();
			return TimeSpent + "," + planLength;
		}
		catch(InvalidConfigurationException e) {
			return "";
		}
	}
    /**
     * Search a solution plan for a planning problem using a Monte-Carlo search strategy.
     *
     * @param problem the problem to solve.
     * @return a plan solution for the problem or null if there is no solution
     */
	public Plan monteCarlo(Problem problem) {
		StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);
		State init = new State(problem.getInitialState());
		Node n = new Node(init, null, -1, 0, 0, heuristic.estimate(init, problem.getGoal()));
		double hMin = n.getHeuristic();
		int counter = 0;
		while (!n.satisfy(problem.getGoal())) {
			if (counter >= MAX_STEPS || getActions(problem, n).isEmpty()) {
				n = new Node(init, null, -1, 0, 0, heuristic.estimate(init, problem.getGoal()));
				counter = 0;
			}
			n = randomWalkAlgo(problem, n, heuristic);
			if (n.getHeuristic() < hMin) {
				hMin = n.getHeuristic();
				counter = 0;
			} else {
				counter++;
			}
		}
		return extractPlan(n, problem);
	}

    /**
     * Extracts a search from a specified node.
     *
     * @param node    the node.
     * @param problem the problem.
     * @return the search extracted from the specified node.
     */
	private Plan extractPlan(final Node node, final Problem problem) {
		Node n = node;
		final Plan plan = new SequentialPlan();
		while (n.getAction() != -1) {
			final Action a = problem.getActions().get(n.getAction());
			plan.add(0, a);
			n = n.getParent();
		}
		return plan;
	}

	/**
	 * The pure random walk algorithm from the "Monte-Carlo Exploration for Deterministic Planning" paper.
	 * @param p the problem
	 * @param s the node
	 * @param heuristic the heuristic
	 * @return the new node
	 */
	public Node randomWalkAlgo(Problem p, Node s, StateHeuristic heuristic) {
		double hMin = Double.MAX_VALUE;
		Node sMin = null;
		for (int i = 0; i < NUM_WALK; i++) {
			Node sPrim = s;
			for (int j = 1; j < LENGTH_WALK; j++) {
				List<Action> A = this.getActions(p, sPrim);
				if (A.isEmpty())
					break;
				Action a = pickRandomAction(A);
				sPrim = putAction(p, sPrim, a, heuristic);
				if (sPrim.satisfy(p.getGoal()))
					return sPrim;
			}
			if (sPrim.getHeuristic() < hMin) {
				hMin = sPrim.getHeuristic();
				sMin = sPrim;
			}
		}
		return sMin == null ? s : sMin;
	}

	/**
	 * Get all applicable actions from a node.
	 * @param p the problem
	 * @param n the node
	 * @return a list of applicable actions
	 */
	private List<Action> getActions(Problem p, Node n) {
		List<Action> actions = p.getActions();
		List<Action> applicableActions = new ArrayList<>();
		for (Action a : actions)
			if (a.isApplicable(n))
				applicableActions.add(a);
		return applicableActions;
	}

	/**
	 * Pick a random Action in a list of Actions
	 * @param listActions a list of Actions
	 * @return a random Action
	 */
	private Action pickRandomAction(List<Action> listActions) {
		Collections.shuffle(listActions);
		return listActions.get(0);
	}

	/**
	 * Apply an action to a node and return the new node.
	 * @param p the problem
	 * @param n the node
	 * @param a the action
	 * @param heuristic the heuristic
	 * @return the new node
	 */
	public Node putAction(Problem p, Node n, Action a, StateHeuristic heuristic) {
		State s = new State(n);
		s.apply(a.getConditionalEffects());
		Node child = new Node(s, n, p.getActions().indexOf(a), n.getCost() + 1, n.getDepth() + 1, 0);
		child.setHeuristic(heuristic.estimate(child, p.getGoal()));
		return child;
	}

	/**
	 * Returns if a specified problem is supported by the planner. Just ADL problem can be solved by this planner.
	 *
	 * @param problem the problem to test.
	 * @return <code>true</code> if the problem is supported <code>false</code> otherwise.
	 */
	@Override
	public boolean isSupported(Problem problem) {
		return (problem.getRequirements().contains(RequireKey.ACTION_COSTS)
				|| problem.getRequirements().contains(RequireKey.CONSTRAINTS)
				|| problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
				|| problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
				|| problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
				|| problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
				|| problem.getRequirements().contains(RequireKey.FLUENTS)
				|| problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
				|| problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
				|| problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
				|| problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
				|| problem.getRequirements().contains(RequireKey.PREFERENCES)
				|| problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
				|| problem.getRequirements().contains(RequireKey.HIERARCHY))
				? false : true;
	}

}

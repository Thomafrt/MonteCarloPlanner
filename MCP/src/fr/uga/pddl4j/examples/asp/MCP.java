package fr.uga.pddl4j.examples.asp;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
import fr.uga.pddl4j.planners.Statistics;
import fr.uga.pddl4j.planners.statespace.HSP;
import fr.uga.pddl4j.problem.*;
import fr.uga.pddl4j.planners.InvalidConfigurationException;
import fr.uga.pddl4j.problem.operator.Action;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;
import fr.uga.pddl4j.parser.RequireKey;

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
	 * The weight of the heuristic.
	 */
	private double heuristicWeight = 1;

	/**
	 * The name of the heuristic used by the planner.
	 */
	private StateHeuristic.Name heuristicName = StateHeuristic.Name.FAST_FORWARD;

	/**
	 * 
	 */
	public static long NUM_WALK = 700;

	/**
	 * 
	 */
	public static long LENGTH_WALK = 7;

	/**
	 * 
	 */
	public static long MAX_STEPS = 7;

	/**
	 * Returns the name of the heuristic used by the planner to solve a planning problem.
	 *
	 * @return the name of the heuristic used by the planner to solve a planning problem.
	 */
	public final StateHeuristic.Name getHeuristic() {
		return this.heuristicName;
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
		LOGGER.info("* Algo MCP a débuté \n");
		final long begin = System.currentTimeMillis();
		final Plan plan = this.monteCarlo(problem);
		final long end = System.currentTimeMillis();
		if (plan != null) {
			LOGGER.info("* Recherche MCP a réussi \n");
			this.getStatistics().setTimeToSearch(end - begin);
		} else {
			LOGGER.info("* Recherche MCP a échoué \n");
		}
		return plan;
	}

	/**
	 * 
	 */
	public Plan monteCarlo(Problem problem){
		StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);
		State init = new State(problem.getInitialState());
		Node n = new Node(init, null, -1, 0, 0, heuristic.estimate(init, problem.getGoal()));
		double hMin = n.getHeuristic();
		int counter = 0;
		while (!n.satisfy(problem.getGoal())) {
			if (counter >= MAX_STEPS || getApplicableActions(problem, n).isEmpty()) {
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
	 * 
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
	 *
	 */
	public Node randomWalkAlgo(Problem p, Node s, StateHeuristic heuristic) {
		/**
		 * L'heuristique poids minimal. On l'initialise à +infini pour être sûr de
		 * trouver un minimum.
		 */
		double hMin = Double.MAX_VALUE;
		/**
		 * sMin est le Node associé à cette heuristique minimale.
		 */
		Node sMin = null;
		/**
		 * On fait un certain nombre de "Walks" pour explorer le voisinage de s.
		 * NUM_WALK modélise la largeur du voisinage de s.
		 */
		for (int i = 0; i < NUM_WALK; i++) {
			Node sPrim = s;
			/**
			 * Pour chaque "walk", on détermine un nombre de pas. Un "walk" consiste
			 * d'aller d'un noeud au suivant LENGTH_WALK fois. LENGTH_WALK modélise la
			 * profondeur du voisinage.
			 */
			for (int j = 1; j < LENGTH_WALK; j++) {
				/**
				 * Pour chaque transition de s', on cherche aléatoirement une action applicable à s' parmi toutes les
				 * actions pour trouver s''.
				 */

				/**
				 * On récupère ici les actions applicables à s' grâce au Problem. Si on n'en
				 * trouve aucune, c'est que ce noeud s' n'a pas de noeud successeur s''. On
				 * passe donc au "Walk" d'après.
				 */
				List<Action> A = this.getApplicableActions(p, sPrim);
				if (A.isEmpty())
					break;

				/**
				 * On choisit aléatoirement une Action applicable à s' pour trouver s''.
				 */
				Action a = UniformlyRandomSelectFrom(A);
				sPrim = apply(p, sPrim, a, heuristic);

				if (sPrim.satisfy(p.getGoal()))
					return sPrim;
			}
			/**
			 * Au bout du voisinage, on calcule l'heuristique de sPrim. Il s'agit d'un
			 * calcul qui approxime la distance entre sPrim et l'état Goal.
			 * Si la valeur de l'heuristique est meilleure (plus petite), on met à jour hMin
			 * et sMin.
			 */
			if (sPrim.getHeuristic() < hMin) {
				hMin = sPrim.getHeuristic();
				sMin = sPrim;
			}
		}

		/**
		 * Retourne le meilleur Node du voisinage de s. S'il n'y en a pas, retourne s.
		 */
		return sMin == null ? s : sMin;
	}

	/**
	 * Retourne les actions applicables au Node en paramètre suivant un Problem.
	 * Peut
	 * retourner une Liste d'Action vide, mais jamais null.
	 *
	 * @param p Le problem en cours.
	 * @param n Un noeud.
	 * @return une List d'Actions.
	 */
	private List<Action> getApplicableActions(Problem p, Node n) {
		List<Action> actions = p.getActions();
		List<Action> applicableActions = new ArrayList<>();
		for (Action a : actions)
			if (a.isApplicable(n))
				applicableActions.add(a);
		return applicableActions;
	}

	/**
	 * Choisis aléatoirement (suivant loi uniforme) une Action parmi une List
	 * d'Actions.
	 *
	 * @param listActions Une List d'Actions.
	 * @return Une Action choisie aléatoirement dans la liste.
	 */
	private Action UniformlyRandomSelectFrom(List<Action> listActions) {
		Collections.shuffle(listActions);
		return listActions.get(0);
	}

	/**
	 * Applique une action au Node en paramètre pour renvoyer le Node résultant.
	 * @param p Le Problem.
	 * @param n Le Node.
	 * @param a L'action appliquée sur n.
	 * @param heuristic L'heuristique utilisée.
	 * @return Le Noeud fils de n après l'action a.
	 */
	public Node apply(Problem p, Node n, Action a, StateHeuristic heuristic) {
		State s = new State(n);
		s.apply(a.getConditionalEffects());
		Node enfant = new Node(s, n, p.getActions().indexOf(a), n.getCost() + 1, n.getDepth() + 1, 0);
		enfant.setHeuristic(heuristic.estimate(enfant, p.getGoal()));
		return enfant;
	}

	/**
	 * The main method of the <code>MCP</code> planner.
	 *
	 * @param args the arguments of the command line.
	 */
	public static void main(String[] args) throws IOException {
		try {
			final MCP mrwPlanner = new MCP();
			final HSP hspPlanner = new HSP();
			File resultFile = new File("src/pddl/data.csv");
			BufferedWriter writer = new BufferedWriter(new FileWriter(resultFile));
			writer.write("domain,problem_number,MCP_time,MCP_length,HSP_time,HSP_length");
			writer.newLine();


			List<File> blocks_problems = List.of(new File("src/pddl/blocks").listFiles());
			//List<File> depot_problems = List.of(new File("src/pddl/depot").listFiles());
			List<File> gripper_problems = List.of(new File("src/pddl/gripper").listFiles());
			List<File> logistics_problems = List.of(new File("src/pddl/logistics").listFiles());

			Map<File, List<File>> pddlFiles = new TreeMap<>();
			pddlFiles.put(new File("src/pddl/blocks_domain.pddl"), blocks_problems);
			//pddlFiles.put(new File("src/pddl/depot_domain.pddl"), depot_problems);
			pddlFiles.put(new File("src/pddl/gripper_domain.pddl"), gripper_problems);
			pddlFiles.put(new File("src/pddl/logistics_domain.pddl"), logistics_problems);

			for(File domainFile : pddlFiles.keySet()) {
				for(File problemFile : pddlFiles.get(domainFile)) {
					String domainPath = domainFile.getPath();
					String problemPath = problemFile.getPath();
					mrwPlanner.setDomain(domainPath);
					hspPlanner.setDomain(domainPath);
					mrwPlanner.setProblem(problemPath);
					hspPlanner.setProblem(problemPath);

					String mrwResults = launch(mrwPlanner);
					String hspResults = launch(hspPlanner);

					String domain = domainFile.getName();
					domain = domain.substring(7, domainFile.getName().lastIndexOf("."));
					String problem = String.valueOf(pddlFiles.get(domainFile).indexOf(problemFile) + 1);
					writer.write(domain + "," + problem + "," + mrwResults + "," + hspResults);
					writer.newLine();
				}
			}

			writer.close();

		} catch (IllegalArgumentException e) {
			LOGGER.fatal(e.getMessage());
		}
	}

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

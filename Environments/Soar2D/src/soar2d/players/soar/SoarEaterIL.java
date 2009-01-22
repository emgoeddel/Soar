package soar2d.players.soar;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import sml.Agent;
import sml.FloatElement;
import sml.Identifier;
import sml.IntElement;
import sml.StringElement;
import soar2d.Names;
import soar2d.Simulation;
import soar2d.map.CellObject;
import soar2d.map.GridMap;
import soar2d.players.Player;

class SoarEaterIL {
	private static Logger logger = Logger.getLogger(SoarEaterIL.class);
	
	private class RandomIL {
		private FloatElement randomWME;

		private void create() {
			randomWME = agent.CreateFloatWME(agent.GetInputLink(), Names.kRandomID, Simulation.random.nextFloat());
		}
		
		private void update() {
			// force a blink:
			this.destroy();
			this.create();
		}
		
		private void destroy() {
			agent.DestroyWME(randomWME);
		}
	}
	
	class MyLocationIL {
		class Cell {
			/** current cell id */
			Identifier me;
			
			/** another eater in the cell */
			StringElement eater;
			
			/** property: edible */
			HashMap<String, StringElement> comestibles = new HashMap<String, StringElement>();
			
			/** static content of the cell */
			HashMap<String, StringElement> staticContent = new HashMap<String, StringElement>();
			
			/** box in current cell, null if none */
			Identifier box;
			
			/** properties on the box if there is one */
			HashMap<String, StringElement> boxProperties = new HashMap<String, StringElement>();

			/** id (likely shared) to the cell/wme to the north */
			Identifier north;
			
			/** id (likely shared) to the cell/wme to the south */
			Identifier south;
			
			/** id (likely shared) to the cell/wme to the east */
			Identifier east;
			
			/** id (likely shared) to the cell/wme to the west */
			Identifier west;
			
			/** used during initialization */
			boolean iterated = false;
		}
		
		/** the vision grid */
		private Cell[][] cells;
		int size;
		int center;
		
		private MyLocationIL(int vision) {
			size = vision * 2;
			size += 1; // for center cell
			center = size / 2; // index of center cell
		}
		
		private void create() {
			cells = new Cell[size][size];
			for (int i = 0; i < cells.length; ++i) {
				for (int j = 0; j < cells.length; ++j) {
					cells[i][j] = new Cell();
				}
			}

			cells[center][center].me = agent.CreateIdWME(agent.GetInputLink(), Names.kMyLocationID);
			createView(center, center);
		}
		
		/**
		 * @param x current cell x location to init
		 * @param y current cell y location to init
		 * 
		 * Recursive function to initialize the vision grid.
		 */
		private void createView(int x, int y) {
			if (x < 0 || x >= cells.length || y < 0 || y >= cells.length || cells[x][y].iterated) {
				return;
			}
			
			cells[x][y].iterated = true;

			if (x > 0) {
				if (cells[x - 1][y].me == null)
					cells[x - 1][y].me = agent.CreateIdWME(cells[x][y].me, Names.kWest);
				else
					cells[x][y].west = agent.CreateSharedIdWME(cells[x][y].me, Names.kWest, cells[x - 1][y].me);
			}
			
			if (x < cells.length - 1) {
				if (cells[x + 1][y].me == null)
					cells[x + 1][y].me = agent.CreateIdWME(cells[x][y].me, Names.kEast);
				else
					cells[x][y].east = agent.CreateSharedIdWME(cells[x][y].me, Names.kEast, cells[x + 1][y].me);
			}
			
			if (y > 0) {
				if (cells[x][y - 1].me == null)
					cells[x][y - 1].me = agent.CreateIdWME(cells[x][y].me, Names.kNorth);
				else
					cells[x][y].north = agent.CreateSharedIdWME(cells[x][y].me, Names.kNorth, cells[x][y - 1].me);
			}
			
			if (y < cells.length - 1) {
				if (cells[x][y + 1].me == null)
					cells[x][y + 1].me = agent.CreateIdWME(cells[x][y].me, Names.kSouth);
				else
					cells[x][y].south = agent.CreateSharedIdWME(cells[x][y].me, Names.kSouth, cells[x][y + 1].me);
			}
			
			createView(x - 1, y);
			createView(x + 1, y);
			createView(x, y - 1);
			createView(x, y + 1);
		}
		
		private void createContent(HashMap<String, StringElement> map, Cell cell, String name) {
			// create the wme
			StringElement element = agent.CreateStringWME(cell.me, Names.kContentID, name);
			assert element != null;
			
			// store the element
			map.put(name, element);
		}
		
		private void updatePlayerContent(int[] view, GridMap map, Cell cell) {
			// TODO: Keep track of player name so we can blink if it is a different eater.
			Player playerContent = map.getPlayer(view);
			if (playerContent != null) {
				if (cell.eater == null) {
					cell.eater = agent.CreateStringWME(cell.me, Names.kContentID, Names.kEaterID);
				}
			} else {
				// Remove any if there
				if (cell.eater != null) {
					agent.DestroyWME(cell.eater);
					cell.eater = null;
				}
			}
		}
		
		private void updateFoodContent(int[] view, GridMap map, Cell cell) {
			// Food
			HashMap<String, StringElement> remaining = new HashMap<String, StringElement>(cell.comestibles);
			// For each food type in the cell on the map
			for (CellObject comestible : map.getAllWithProperty(view, Names.kPropertyEdible)) {
				
				String id = comestible.getProperty(Names.kPropertyID);
				
				// Do we have one?
				if (cell.comestibles.containsKey(id)) {
					// Keep it and remove it from the remaining
					remaining.remove(id);
				} else {
					
					// Create it and move on
					createContent(cell.comestibles, cell, id);
				}
			}
			
			// Remove all remaining
			for (Entry<String, StringElement> entry : remaining.entrySet()) {
				// Remove it from the main list
				cell.comestibles.remove(entry.getKey());
				
				// Destroy the WME
				agent.DestroyWME(entry.getValue());
			}

		}
		
		private void updateBoxProperties(CellObject box, Cell cell) {
			HashMap<String, StringElement> remaining = new HashMap<String, StringElement>(cell.boxProperties);
			// For each box property
			for (String property : box.getPropertyNames()) {
				// Do we have it?
				if (cell.boxProperties.containsKey(property)) {
					// Keep it and remove it from the remaining
					remaining.remove(property);
				} else {
					StringElement element = agent.CreateStringWME(cell.box, property, box.getProperty(property));
					cell.boxProperties.put(property, element);
				}
			}
			
			// Remove all remaining
			for (Entry<String, StringElement> entry : remaining.entrySet()) {
				// Remove it from the main list
				cell.boxProperties.remove(entry.getKey());
				
				// Destroy the WME
				agent.DestroyWME(entry.getValue());
			}
		}
		
		private void updateBox(CellObject box, Cell cell) {
			if (box != null) {
				if (cell.box == null) {
					// create the wme
					String id = box.getProperty(Names.kPropertyID);
					if (id == null) {
						id = Names.kDefaultBoxID;
					}
					cell.box = agent.CreateIdWME(cell.me, id);
					assert cell.box != null;
					
					// go through the properties and add them
					for (Entry<String, String> entry : box.getPropertyEntries()) {
						StringElement element = agent.CreateStringWME(cell.box, entry.getKey(), entry.getValue());
						cell.boxProperties.put(entry.getKey(), element);
					}
				} else {
					updateBoxProperties(box, cell);
				}
				
			} else {
				if (cell.box != null) {
					agent.DestroyWME(cell.box);
					cell.box = null;
					
					for (StringElement property : cell.boxProperties.values()) {
						agent.DestroyWME(property);
					}
					cell.boxProperties.clear();
				}
			}
		}
		
		private void checkEmpty(Cell cell) {
			// empty test
			// a cell is empty if it doesn't have food, a player, or a box
			// wall is implied since we can't get here if there is a wall
			if (cell.comestibles.isEmpty() && cell.eater == null && cell.box == null) {
				if (!cell.staticContent.containsKey(Names.kEmpty)) {
					createContent(cell.staticContent, cell, Names.kEmpty);
				}
			} else {
				StringElement element = cell.staticContent.remove(Names.kEmpty);
				if (element != null) {
					agent.DestroyWME(element);
				}
			}
		}
		
		private void update(boolean moved, int[] pos, GridMap map) {
			if (moved) {
				destroy();
				create();
			}

			int[] view = new int[2];
			for (int i = 0; i < cells.length; ++i) {
				view[0] = pos[0] - center + i;
				for (int j = 0; j < cells.length; ++j, ++view[1]) {
					view[1] = pos[1] - center + j;
					
					Cell cell = cells[i][j];
					
					if (moved) {
						
						// if out of bounds, create wall
						if (!map.isInBounds(view)) {
							createContent(cell.staticContent, cell, Names.kWallID);
							
							// nothing else to do
							continue;
						}
						
						// Blocking cells are simple, put anything with IDs on the input link
						if (!map.enterable(view)) {
							// get all things that block
							for (CellObject object : map.getAllWithProperty(view, Names.kPropertyBlock)) {
								// use the id property as its id on the input link
								createContent(cell.staticContent, cell, object.getProperty(Names.kPropertyID));
							}
							continue;
						}
					} else {
						
						// Filter out locations that will not change:
						if (!map.isInBounds(view) || !map.enterable(view)) {
							continue;
						}
					}
					
					// Create/update content algorithm is not perfect. Items that have the same name will
					// be detected as the same item and not blink even if they should.
					
					updatePlayerContent(view, map, cell);
					updateFoodContent(view, map, cell);
					
					// TODO: there can only be one (as of right now)
					ArrayList<CellObject> boxes = map.getAllWithProperty(view, Names.kPropertyBox);
					assert boxes.size() <= 1;
					CellObject box = boxes.size() > 0 ? boxes.get(0) : null;
					
					updateBox(box, cell);
					
					checkEmpty(cell);
				}
			}
		}
		
		private void destroy() {
			int center = cells.length / 2;
			agent.DestroyWME(cells[center][center].me);
			cells = null;
		}
	}
	
	private class EaterIL {
		Identifier eaterWME;
		StringElement nameWME;
		IntElement scoreWME;
		IntElement[] posWMEs;
		
		void create(String name, int initialScore) {
			eaterWME = agent.CreateIdWME(agent.GetInputLink(), Names.kEaterID);
			scoreWME = agent.CreateIntWME(eaterWME, Names.kScoreID, initialScore);
			posWMEs = new IntElement[] { 
					agent.CreateIntWME(eaterWME, Names.kXID, 0), 
					agent.CreateIntWME(eaterWME, Names.kYID, 0)
			};
			nameWME = agent.CreateStringWME(eaterWME, Names.kNameID, name);
		}
		
		void update(boolean moved, int[] pos, int points) {
			agent.Update(scoreWME, points);
			agent.Update(posWMEs[0], pos[0]);
			agent.Update(posWMEs[1], pos[1]);
			
		}
		
		void destroy() {
			if (!agent.DestroyWME(eaterWME)) {
				logger.trace("destroy eaterWME failed");
			}
			
			eaterWME = null;
			scoreWME = null;
			posWMEs = null;
			nameWME = null;
		}
	}
	
	private Agent agent;
	private EaterIL eater = new EaterIL();
	private MyLocationIL myLocation;
	private RandomIL random = new RandomIL();

	SoarEaterIL(Agent agent, int vision) {
		this.agent = agent;
		myLocation = new MyLocationIL(vision);
	}
	
	void create(String name, int initialScore) throws CommitException {
		eater.create(name, initialScore);
		myLocation.create();
		random.create();
		
		if (!agent.Commit()) {
			throw new CommitException();
		}
	}
	
	void update(boolean moved, int[] pos, GridMap map, int points) throws CommitException {
		eater.update(moved, pos, points);
		myLocation.update(moved, pos, map);
		random.update();
		
		if (!agent.Commit()) {
			throw new CommitException();
		}
	}
	
	void destroy() throws CommitException {
		eater.destroy();
		myLocation.destroy();
		random.destroy();
		
		if (!agent.Commit()) {
			throw new CommitException();
		}
	}
}

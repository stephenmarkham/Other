/**
 * decrypt.java
 * Date: 5/09/2015
 * Author: Stephen Markham
 * Version: 1.0
 *
 * Program to decrypt the Permutation cipher text for COSC 412 Assignment 1
 *
 * run by calling
 *                  java decrypt [filename.txt]
 */

import java.util.*;
import java.io.*;
 
public class decrypt{

	/**
     * Main Method
     */
    public static void main(String[]args){
 		
 		//If file is in args
        if (args.length == 1){

	        try{
	        	//Opens file, starts scanner
		        File f = new File(args[0]);
		        Scanner s = new Scanner(f);

		        // Stores file in String
		        String text = s.nextLine();

		        //for key lengths up to 20 (stops when solution found)
		        for (int i = 2; i <= 20; i++){
		    		System.out.printf("------------ Key Length: %d ----------------\n", i);
		    		// Calls the decrypt function with text and key length
		    		if (decrypt(text, i)) break;
		        }
		    //Error Opening File
		    }catch(Exception e){
		    	System.err.println("\nError: Failed to Open File\n");
		    }

		// Wrong Command Line Arguments   
		}else{
			System.err.println("\nError: No File Name\n\nRun Using\n   java decrypt [filename.txt]\n");
		}
    }

    
    /**
	 * decrypt
	 * Method that decrypts the text for a given key length
	 *
	 * @param String text, the text to decrypt
	 * @param int keyLength, the key length used to decrypt
	 * @return boolean, true if successful, false if no decryption found
	 */
    public static boolean decrypt(String text, int keyLength){
    	//Number of Rows when the text is placed in a table with width = keyLength 
    	int depth =  (int) Math.ceil((double)text.length() / keyLength);

    	//Stores values relating to the probability 2 columns are next to each other
    	double[][] nextTo = new double[keyLength][keyLength];

    	//Stores the text into a char[][][] array
    	char[][][] table = buildTable(text, keyLength);

    	//Loops through all possible ordering of columns
    	//and checks to see if bi-gram frequencies are similar to English text
    	for (int i = 0; i < keyLength; i++){
    		for (int j = 0; j < keyLength; j++){
    			double total = 0;
    			for (int k = 0; k < depth-1; k++){//ignores last line as may not be complete
    				//Checks if bi-gram ab is regular
    				char a = table[i][k][0];
    				char b = table[j][k][0];
    				total += getTableValue(a,b);
    			}
    			//Average of the column
    			nextTo[i][j] = total/depth;
    		}

    	}

    	//Finds the column pairs that may match
    	ArrayList<Integer> matches = new ArrayList<Integer>();
    	for(int i = 0; i < keyLength; i++){
    		for (int j = 0; j < keyLength; j++){
    			if (nextTo[i][j] > 0){
    				matches.add(i);
    				matches.add(j);
    			}		
    		}
    	}

    	//if the number of matching pairs is complete
    	if(matches.size() > (keyLength*2)-3){
    		
    		//Sort the pairs into a list of column orders
    		int[] ordered = new int[keyLength];
    		ordered = orderArray(matches);
    		
    		System.out.println("Match found\n");
    		
    		//Prints out the text based upon the column ordering
    		for (int i = 0; i < depth; i++){
	    		for (int j = 0; j < keyLength; j++){
	    			int k = ordered[j];
	    			System.out.print(table[k][i][0]);
	    		}
	    	}

	    	System.out.println("\n");

	    	//returns true to say a match has been found
	    	return true;

    	}else {
    		// No Match return false
    		System.out.println("No Match");
    		return false;
    	}
    }
    

    /**
	 * buildTable
	 * Sorts the input text into the rows and columns used during the decryption process
	 *
	 * @param String text, the input text
	 * @param int keyLength, the key length used to decrypt
	 * @return char[][][], the char array used to decrypt
	 */
    public static char[][][] buildTable(String text, int keyLength){

    	//Number of Rows when the text is placed in a table with width = keyLength 
    	int depth =  (int) Math.ceil((double)text.length() / keyLength);

    	// Table to store text
    	char[][][] table = new char[keyLength][depth][1];

    	//Reads text into table
    	int index;
    	for (int i = 0; i < keyLength; i++){
    		index = i;
    		for (int j = 0; j < depth; j++){
    			if (index >= text.length()) break;
    			else table[i][j][0] = text.charAt(index);
    			index += keyLength;
    		}
    	}

    	return table;
    }

    /**
	 * getTableValue
	 * retrieves the value from the lookup table relating to the frequency of letters found next to each other
	 *
	 * @param char a, the first letter
	 * @param char b, the second letter
	 * @return int, the value relating to the frequency that a and b are found as a bi-gram
	 */
    public static int getTableValue(char a, char b){
    	
    	// turn char values into number in the alphabet (0 start)
    	int aVal = (Character.getNumericValue(a)) - 10;
    	int bVal = (Character.getNumericValue(b)) - 10;

    	//return the ab bi-gram frequency value from look up table
    	return LOOKUP[aVal][bVal];
    } 

    /**
	 * orderArray
	 * Given the pairs of values, it orders them into the order that the columns should be in
	 * example conversion [0 2  4 5  5 0] -> [4 5 0 2]
	 *
	 * @param ArrayList<Integer> matches, matching pairs found
	 * @return int[], the order in which the columns are read
	 */
    public static int[] orderArray(ArrayList<Integer> matches){
    	
    	//Place to store order list
    	ArrayList<Integer> ordered = new ArrayList<Integer>();

    		// While there is still stuff to process
	    while(!matches.isEmpty()){
	   		//if nothing is in the ordered array, add the first to value to start
	   		if (ordered.isEmpty()){
	   			//add values
	   			ordered.add(matches.get(0));
	   			ordered.add(matches.get(1));
	   			//Remove the 2 values from the original
	   			matches.remove(0);
	   			matches.remove(0);
	   		}

    		// Move forward through the list and add any that fit
    		for (int i = 0; i < matches.size(); i+= 2){
               //if first of pair in in ordered
    			if (ordered.contains(matches.get(i))){
                    //add the next
	    			ordered.add(ordered.indexOf(matches.get(i))+1, matches.get(i+1));
                    //remove both pair
	    			matches.remove(i);
	    			matches.remove(i);
                    //fix iterator seems removed 2 elements
	    			i-=2;
	    		}
	    	}

	    	// Move backward though the list and add any that fit
	    	for (int i = matches.size()-1; i >= 0; i-= 2){
                //if second of pair in in ordered
	   			if (ordered.contains(matches.get(i))){
	    			// add 1 value
	   				ordered.add(ordered.indexOf(matches.get(i)), matches.get(i-1));
	    			//remove the 2 values
	    			matches.remove(i-1);
	    			matches.remove(i-1);
	    		}
	    	}
	   	}

	    //Convert to int[]
	    int[] intArray = new int[ordered.size()];
		for (int i = 0; i < intArray.length; i++) {
    		intArray[i] = Integer.valueOf(ordered.get(i));
    	}

    	// return the ordered array
	    return intArray;
    }

    /**
     * Lookup table with frequecy values for all possible bi-grams
     * retrieved from 
     * https://cdn.fbsbx.com/hphotos-xtf1/v/t59.2708-21/11640721_10205740399660788_143010376_n.pdf/breaking_tranposition_cipher.pdf?oh=f090931f45f190a9f03c823415cdc791&oe=55ED710F&dl=1
     *
     */                         //a, b, c, etc
    static int[][] LOOKUP = { 	{-8, 1, 2, 1,-8,-1, 0,-7,-1,-2, 2, 2, 1, 3,-8, 1,-1, 1, 2, 1,-3, 3,-1,-2, 1, 2},//a
 								{ 0,-2,-8,-8, 3,-8,-8,-8,-1, 3,-8, 3,-6,-8, 1,-8,-8, 1,-4,-7, 5,-8,-8,-8, 4,-8},//b
 								{ 1,-8,-1,-8, 1,-8,-8, 3,-1,-8, 6, 0,-8,-8, 3,-8, 3, 0,-8,-1, 0,-8,-8,-8,-4,-2},//c
 								{ 1, 3, 0,-2, 0, 0,-1,-1, 1, 3,-3,-1, 0,-2, 0,-1, 0,-2, 0, 1,-1,-2, 1,-8, 0,-8},//etc
 								{ 0, 1, 1, 2,-3, 0,-2,-3,-2, 0,-2, 0, 1, 1,-3,-1, 2, 3, 1,-1,-5, 2, 1, 5, 0,-1},
 								{ 1,-1,-2,-5,-1, 2,-4,-2, 1, 1,-7, 0, 0,-8, 2,-1,-3, 1,-3, 2, 1,-4,-1,-8,-2,-6},
 								{ 1, 0,-3,-5, 0,-1, 0, 3, 1, 1,-7, 0,-2,-4, 1,-3,-3, 0,-1,-1, 1,-4, 0,-8,-4,-8},
 								{ 2,-5,-7,-8, 4,-6,-8,-5, 2,-3,-8,-8,-5,-8, 0,-6,-5,-4,-6,-2,-2,-8,-4,-8,-3,-8},
 								{-5,-2, 2, 0,-4, 0, 2,-6,-8,-1, 1, 1, 2, 4,-2,-2,-2,-1, 2, 1,-8, 3,-2, 2,-8, 5},
 								{ 1,-8,-8,-8, 0,-8,-8,-8, 2,-5,-8,-8,-8,-8, 2,-8,-8,-8,-8,-8, 8,-8,-8,-8,-8,-8},
 								{ 0, 0,-4,-6, 3,-1,-6,-3, 3, 0,-8,-2,-3, 1,-1,-4,-2,-7, 0,-2,-3,-7, 0,-8, 0,-8},
 								{ 1,-2,-4, 2, 1, 0,-5,-5, 2,-3, 0, 4,-3,-7, 1,-2,-3,-6,-2,-3,-1,-1,-2,-8, 4, 1},
 								{ 2, 2,-5,-8, 2,-3,-6,-5, 1,-1,-7,-6,-1,-7, 1, 3,-4,-4,-1,-3, 0,-8,-2,-8, 4,-8},
 								{-1,-1, 1, 4,-1,-2, 5,-3,-1, 1, 1,-3,-3,-5, 0,-3, 1,-8,-1, 1,-4,-3,-1,-3,-1,-3},
 								{-4, 0,-1,-2,-8, 4,-2,-4,-4,-1, 2, 0, 3, 2,-1, 1,-2, 2,-2,-1, 5, 2, 3,-2,-3, 0},
 								{ 2,-4,-7,-8, 1,-5,-8,-3, 0,-3,-8, 3,-5,-8, 2, 4,-4, 2,-2,-1, 1,-8,-3,-8,-3,-8},
 								{-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8,-8, 8,-8,-8,-8,-8,-8},
 								{ 0,-1,-1,-1, 2,-1, 0,-3, 1, 0, 1,-2, 0,-2, 1,-1, 0,-2, 0,-1,-1, 0,-1,-3, 2,-8},
 								{ 1, 1, 0,-4, 0,-1,-3, 1, 0, 1, 0,-2,-1,-4, 1, 2, 3,-6, 0, 2, 0,-3, 1,-8,-3, 0},
 								{-1,-1,-2,-5,-1,-3,-5, 5, 1, 0,-4,-2,-2,-6, 1,-4,-2,-2,-2,-1,-1,-7, 1,-8, 0,-5},
 								{-4, 0, 2,-2,-5,-3, 2,-7,-3,-3,-2, 3, 0, 2,-8, 4,-6, 3, 2, 2,-8,-5,-3,-4,-7, 2},
 								{-1,-8,-8,-8, 5,-8,-8,-8, 2,-8,-8,-8,-8,-8,-1,-8,-8,-8,-8,-8,-8,-8,-8,-8,-3,-8},
 								{ 3,-4,-5,-6, 1,-5,-8, 3, 2,-2,-6,-5,-4,-1, 1,-6,-6,-5,-4,-5,-7,-7,-3,-8,-4,-4},
 								{ 0,-4, 5,-7,-1,-3,-8,-2, 2,-8,-8,-8,-3,-8,-5, 7, 4,-8,-5, 2,-3, 0,-3, 6,-2,-8},
 								{ 0, 3, 1,-1,-2, 1,-1,-1, 0, 3,-2,-2, 1,-4, 3, 0, 1,-3, 1, 0,-4,-1, 2,-8,-2,-8},
 								{ 1, 4,-7,-7, 4,-7,-5,-6, 1,-4, 0, 0,-5,-8,-1,-5,-8,-8,-8,-7,-6,-8,-8,-8, 2, 8}};
}
/*
automator
This is a program to automatically startup several tests for AU_UAV_ROS evaluator when the
inputs are enumerated in another .txt file.


It also analyzes the score files created by the evaluator, and creates a new file
with some averages of all the score files.

Compiled with G++.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <string.h>
#include <sys/wait.h>
#include <iostream>
#include <sstream>



//normal time + 30 as buffer time
#define SIMULATION_TIME 300

#define CHECK_LENGTH 5  //how often we check to see if evaluator is done(every 30 sec)

#define BUFFER_TIME 5
#define SLEEP_TIME (SIMULATION_TIME+16)
#define OUTPUT_ADDITION "\n" //the '\n' is CRITICAL
#define LENGTH_OF_EXTENSION 7 //".course" has 7 characters

#define ALL_SCORES_EXTENSION "_scores.txt"
#define DATA_EXTENSION "_data.txt"

#define DONE_FILE "doneWithCourse.txt"

#define DONE_FILE_FULL "/home/monzy/catkin_ws/src/au_uav_ros/courses/doneWithCourse.txt"

#define INPUT_DIRECTORY "/home/monzy/catkin_ws/src/au_uav_ros/courses/"
#define SCORES_DIRECTORY "/home/monzy/catkin_ws/src/au_uav_ros/scores/"
#define DATA_DIRECTORY "/home/monzy/catkin_ws/src/au_uav_ros/scores/"
#define PID_FILE_FULL "/home/monzy/catkin_ws/src/au_uav_ros/courses/pid.txt"

bool notDone = true;

/*
isBlankLine(...)
simple function for parsing to determine is a string is a "blank" line
NOTE: Copied from elsewhere
*/
bool isBlankLine(char str[])
{
	for(int i = 0; i < strlen(str); i++)
	{
		switch(str[i])
		{
			case ' ':
			case '\n':
			case '\t':
			{
				//keep checking
				break;
			}
			default:
			{
				//not a blank line character
				return false;
				break;
			}
		}
	}
	
	//we made it here, must be blank
	return true;
}

static void signal_handler(int signo){
	printf("SIGNAL HANDLED\n");
	notDone = false;
}

int main()
{

	if (signal(SIGUSR1, signal_handler) == SIG_ERR) {
        	printf("SIGINT install error\n");
        	exit(1);
	}

	//now put out pid in file
	FILE * pidFD = fopen(PID_FILE_FULL,"w");
	if(pidFD == NULL){printf("ERROR: NO PID FILE");}
	else{
		fprintf(pidFD,"%d",getpid());
		fclose(pidFD);
	}


    
/* ************ FIRST RUN THE EVALUATOR ON ALL FILES ************** */


	int defaultSleep = 0;
	

	//get the filename that has all the course files listed in it
	//(each filename on its own line in plain text)
	char filename[256];
	printf("Enter the file with all the courses in it: ");
	scanf("%s", filename);

	switch(filename[0]){
		case 'a': 
			defaultSleep = 30;
		case 'd':
			defaultSleep = 30;
		case 'g':
			defaultSleep = 50;
		case 'j':
			defaultSleep = 60;


	}

	std::string inFile = std::string(filename);
	inFile = INPUT_DIRECTORY + inFile;
	
	//open the file
	FILE *fp;
	//fp = fopen(filename, "r");
	fp = fopen(inFile.c_str(),"r");	

	//try to create our pipe for use later
	int pfds[2];
	if(pipe(pfds) == -1)
	{
		perror("Pipe");
		exit(1);
	}

	/*int inPipe[2];
	if(pipe(pfds) == -1)
	{
		perror("Pipe");
		exit(1);
	}*/
	
	int pid;
	//check for a good open
	if(fp != NULL)
	{
		char buffer[256];
		//while we have something in the file
		while(fgets(buffer, sizeof(buffer), fp))
		{	
			//check for useless lines
			if(buffer[0] == '#' || isBlankLine(buffer))
			{
				//this line is a comment
				continue;
			}
			
			//construct our strings to send
			std::string myStr = std::string(buffer);
			myStr = myStr.substr(0, myStr.size() - LENGTH_OF_EXTENSION - 1);
			//unsigned pos = myStr.find_first_of("_");//get the position of the first '_'
			//myStr = myStr.substr(0,pos) + "/" +myStr;
			myStr = myStr + OUTPUT_ADDITION;
			
			//fork our process
			//int pid;
			pid = fork();
	
			if(pid == 0)
			{
				//we're redirecting STDIN such that it comes from the pipe
				//close standard in
				close(STDIN_FILENO);
		
				//duplicate our stdin as the pipe output
				dup2(pfds[0], STDIN_FILENO);
				
				//child process
				system("roslaunch au_uav_ros evaluation.launch");

				//printf("\n\n\nSYSTEM************************\n\n\n");
			}
			else
			{
				//send out output over that there pipe
				printf("Writing to the pipe! %s\n", buffer);
				write(pfds[1], buffer, strlen(buffer));
				printf("Writing to the pipe! %s\n", myStr.c_str());
				write(pfds[1], myStr.c_str(), strlen(myStr.c_str()));

				char time [20];
				sprintf(time,"%d",SIMULATION_TIME);
				//sprintf(time,"%d",simTime);	

				printf("Writing to the pipe! %s\n", time);
				write(pfds[1], time, strlen(myStr.c_str()));	
		
				//parent waits some time, then kills before starting new one
				sleep(defaultSleep);

				//just spin until evaluator sends a signal
				while(notDone){
					sleep(CHECK_LENGTH);
				}
				notDone = true;
				sleep(BUFFER_TIME);//give evaluator time to end

				/*bool notDone = true;
				char buf[3];
				int times = 0;
				while(notDone){
					times++;
					buf[0] = 'g';
					sleep(CHECK_LENGTH);
					FILE *dp = fopen(DONE_FILE_FULL,"r");
					if(dp==NULL){printf("\n\nERROR OPENING DONE FILE!! \n\n");}
					fgets(buf,sizeof(buf),dp);
					fclose(dp);
					
					if(buf[0]=='y'){
						dp = fopen(DONE_FILE_FULL,"w");
						printf("WRITEING TO DONE FILE");
						notDone = false;
						fprintf(dp,"n");
						fclose(dp);
						buf[0] = 'g';
					}
					//fclose(dp);
					if((defaultSleep + (times-1)*CHECK_LENGTH)-20 > SIMULATION_TIME){
						notDone = false;
						break;
					}
					
				}//notDone
		*/



				//waitpid(pid, NULL, 0);
				//pause();
				printf("Killing Process ID #%d\n", pid);
				kill(pid, SIGTERM);
				waitpid(pid, NULL, 0);
				//printf("\n\n\nSYSTEM************************\n\n\n");
				//give the SIGTERM time to work
				sleep(BUFFER_TIME);
			}
		}//while
		
		fclose(fp);
	}//if fp!= null
	else
	{
		printf("ERROR: Bad file name\n");
	}

	



/* ******************  NOW THE SCORE SHEET ANALYSIS ******************* */
	
	//char yn = 'y';	
	//std::string input;	
	//std::cout << "Calculate stats?[y/n]: ";
	//std::getline(std::cin, input);
	//std::cin >> input;
	//yn = input[0];

	if(true){ //dont prompt for now, just DO IT.

		

		/*there is a file, similar to the one with all the course files
		 that lists all score files. The name is generated in the 
		 evaluator, by replacing '.txt' with '_scores.txt' in the 
		 filename that lists all the course files. */

		std::string allScores = std::string(filename);//filename has the name of the course list file
		int pos = allScores.find_first_of(".");
		allScores = allScores.substr(0,pos);	//get rid of the .txt extension
		allScores = allScores + ALL_SCORES_EXTENSION; //now we have the correct name
	
		FILE * allScoresFP;
		allScoresFP = fopen((SCORES_DIRECTORY+allScores).c_str(),"r"); //open the score list file

		if(allScoresFP==NULL){printf("BAD SCORE FILE\n");}
		else{

			/*here  we go through each score file in turn, reading and parsing just the 
			 first line. The evaluator makes the score files, and places the data we need on
 			 the first line, each number separted by a space.The format is

			numConflicts numCollisions totalDistance totalMinimumDistance  */

			FILE *scoreFP;
			char numbers[256];
			int numConflicts=0;
			int numCollisions=0;	
			double totalActualDist=0.0;
			double totalMinDist=0.0;
			int numAvoids = 0;
			double numFiles = 0.0;  //its a double because we divide by it later(dont want truncation)
			std::string allNums;
			int pos;
			char buffer[256];

			//while we have a score file in the list
			while(fgets(buffer, sizeof(buffer), allScoresFP)){  //fgets stops at a newline or size or eof
	
				//check for useless lines
				if(buffer[0] == '#' || isBlankLine(buffer))
				{
				//this line is a comment
					continue;
				}
	

				
				std::string myStr = std::string(buffer);//the name of a score file
				
				myStr = myStr.substr(0,myStr.size()-1);//get rid of the newline
				scoreFP = fopen((SCORES_DIRECTORY + myStr).c_str(),"r");//open the single score file

				if(scoreFP==NULL){printf("Could not open score file.\n");}

				else{
					numFiles++; //count of the number of score files
				
					fgets(numbers,sizeof(numbers),scoreFP);//get the first line of the score file
				
					allNums = (numbers);//put the first line in a c++ string for easy parsing
				

					/*now get each number from the line
					 just get posistion of the first space,
					 and get the substring from the begining to the space and turn that to an integer or double using stdlib
					  finally lose the part of the string before the first space*/

					pos = allNums.find_first_of(" ");
					numConflicts += atoi(allNums.substr(0,pos).c_str());
					allNums = allNums.substr(pos+1);
							
					pos = allNums.find_first_of(" ");
					numCollisions += atoi(allNums.substr(0,pos).c_str());
					allNums = allNums.substr(pos+1);

					pos = allNums.find_first_of(" ");
					totalActualDist += atof(allNums.substr(0,pos).c_str());
					allNums = allNums.substr(pos+1);
				
					pos = allNums.find_first_of(" ");
					totalMinDist += atof(allNums.substr(0,pos).c_str());
					allNums = allNums.substr(pos+1);

					pos = allNums.find_first_of("\n");
					numAvoids += atof(allNums.substr(0,pos).c_str());


					fclose(scoreFP);
				}//else  - single score file opened

			}//while there is another score file	

			fclose(allScoresFP);//close the score list file			


	/* now we average everything out and write it out to a file*/


			double avgConflicts;
			double avgCollisions;
			double avgActualDist;
			double avgMinDist;
			double effiency;
			double avgAvoids;

			avgConflicts = numConflicts/numFiles;
			avgCollisions = numCollisions/numFiles;
			avgActualDist = totalActualDist/numFiles;
			avgMinDist = totalMinDist/numFiles;
			effiency = avgMinDist/avgActualDist; //should be < 1
			avgAvoids = numAvoids/numFiles;



			/* *naming convention* */
			/*same as making score list, but now replace '.txt' from course list
			 with '_data.txt*/

			std::string data = std::string(filename);
			pos = data.find_first_of(".");
			data = data.substr(0,pos);	//get rid of the .txt extension
			data = data + DATA_EXTENSION;
			
			FILE * dataFP;
			dataFP = fopen((DATA_DIRECTORY + data).c_str(),"w");

			fprintf(dataFP,"Average Conflicts: %f\n"
					"Average Collisions: %f\n"
					"Average Distance Traveled: %f\n"
					"Average Minimum Distance needed: %f\n"
					"Average effiency: %f\n"
					"Average Avoids: %f\n",
			           avgConflicts,avgCollisions,avgActualDist,avgMinDist,effiency,avgAvoids);


			fclose(dataFP);
		}//else allScoresFP opened properly
	}//if yes do stats
	kill(pid, SIGTERM);
	return 0;
}

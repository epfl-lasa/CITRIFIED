

#include "NIDAQmx.h"
#include "MatlabLib.h"
#include "filters.h"
#include "ESN.h"
#include "rFunctions.h"
#include "tcpHandle.h"

#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <fstream>
#include <windows.h>
#include <time.h>
#include <vector>
#include <future>





// nimber of EMG channels
enum ddims{ DIMS = 13 };

// vectors to store the networks
//std::vector<ESN> networks;
ESN networks(15);

// choice for publishing the classification outcome or not
char pub_ch;

// time window counter
int cnt = 0;

// trial counter
int trial = 530;

// base directory
std::string base = "data\\";

// name of the folder to be created
std::string folderName;



// vectors to store the data of each time window and for all the duration of the trial
std::vector< std::vector<double> > twData(DIMS);
std::vector< std::vector<double> > allData(DIMS);

// duration of the time window (in seconds)
double twDuration = 0.150;
int smallbuffer = 30;

// the sample rate of the acquuisition		
int32 fs = 1000;

// overlap of the time windows (in seconds)
double ovrlp = 0.050;

double **classif_data = new double*[(int)(twDuration*fs)];;

//classif_data = new double*[(int)(twDuration*fs)];

int classifResult = 0;

int nforget = 30;


std::vector< std::vector<double> > ppSignals;
std::string matFF;

// node to handle the TCP connection
tcpHandle node("128.178.145.79");


int pubTW(std::vector<double> Bbdp, std::vector<double> Abdp, std::vector<double> Blp, std::vector<double> Alp, std::vector<double> MVC, float64 *timeWindow, int nb_samp){

	std::vector< std::vector<double> > saveSignal(DIMS, std::vector<double>((int)(twDuration*fs), 0));

	
	//std::vector< std::vector<double> > twData(DIMS, std::vector<double>(nb_samp,0));
	int cc = 1;
	for (int i = 0; i < DIMS*nb_samp; i++){

		if (i >= cc*nb_samp){
			cc++;
		}
		twData[cc - 1].push_back(timeWindow[i]);
		allData[cc - 1].push_back(timeWindow[i]);
	}
	//std::cout << "size twData: " << twData.size() << "," << twData[0].size()<<"\n";

	if ((int)twData[0].size()>(int)(twDuration*fs - 1)+smallbuffer){

		std::vector< std::vector<double> > tmptwData = twData;

		for (int j = 0; j < DIMS; j++){
			twData[j].erase(twData[j].begin(), twData[j].begin() + (int)((twDuration - ovrlp)*fs) + 1);

		}

		ppSignals = preProcessTW(Bbdp, Abdp, Blp, Alp, MVC, tmptwData);
		matFF = folderName+"\\timeWindows\\tw" + std::to_string(cnt);

		//std::cout << "size preProcessTW: " << ppSignals.size() << "," << ppSignals[0].size() << "," << ppSignals[1].size() << "\n"


		if (pub_ch == 'y'){


			int filteredSignalSize = (int)ppSignals[0].size() - (int)(twDuration*fs);

			for (int i = 0; i < (int)(twDuration*fs); i++){
				classif_data[i] = new double[DIMS];
				for (int j = 0; j < DIMS; j++){
					classif_data[i][j] = ppSignals[j][i + filteredSignalSize];
					saveSignal[j][i] = ppSignals[j][i + filteredSignalSize];
				}
			}
		


			// publish the message
			classifResult = networks.test_esn(classif_data, nforget, twDuration*fs);

			////int ndatapoints = (int)(twDuration*fs);
			//if (cnt < networks.size()){
			//	classifResult = networks[cnt].test_esn(classif_data, nforget, twDuration*fs);
			//}
			//else{
			//	classifResult = networks.back().test_esn(classif_data, nforget, twDuration*fs);
			//}


			//std::cout << classifResult << "\n";
			if (cnt >0 ){	
				if (node.isConnected()){
					node.publish(classifResult);
				}
				else{
					std::cout << "Problem with the TCP connection!!!\nCheck if server is working\n";
				}
			}

		}
		else{
			
			int filteredSignalSize = (int)ppSignals[0].size() - (int)(twDuration*fs);
			//std::cout << "ppSignals " << (int)ppSignals.size() << "filteredSignalSize :" << filteredSignalSize << "\n";
			for (int i = 0; i < (int)(twDuration*fs); i++){
				//std::cout << i << "\n";
				for (int j = 0; j < DIMS; j++){
					
					saveSignal[j][i] = ppSignals[j][i + filteredSignalSize-1];
				}
			}

		}

		saveData(matFF, "twM", saveSignal);


		// shift back the data
		



		cnt++;
		std::cout << cnt << "\n";

	}



	return 0;

}





#define DAQmxErrChk(functionCall) if(DAQmxFailed(error=(functionCall))) goto Error; else
int main(void){

	std::string sbjName;
	std::cout << "Type the id of the subject: ";
	std::cin >> sbjName;
	base += "sbj"+sbjName;
	CreateDirectory(base.c_str(), NULL);

	std::cout << "Do you want to publish the signals? (y/n) ";
	std::cin >> pub_ch;



	if (pub_ch == 'y'){
		int nbESNs = count_esn("data\\networks\\ESN");
		std::cout << "loading ESNs...\n";
		//std::cout << "nets= " << count_esn("data\\networks\\ESN") << "\n";
		for (int i = 1; i <= nbESNs; i++){
			std::cout << ".";
			//networks.push_back(ESN(i));

		}
		std::cout << "\n" << nbESNs << " loaded\n";
	}

	// the mvc for normalization
	std::vector<double> mvc(DIMS, 1);
	mvc[0] = 1.1535e+03; //mvc[1] = 0.8786e+03; mvc[2] = 1.8030e+03; mvc[3] = 1.1400e+03; mvc[4] = 2.2979e+03; mvc[5] = 1.4470e+03; mvc[6] = 0.9786e+03; mvc[7] = 0.9859e+03; mvc[8] = 1.0175e+03; mvc[9] = 1.1539e+03; mvc[10] = 1.7690e+03; mvc[11] = 2.1906e+03; mvc[12] = 1.9659e+03; //mvc[13] = 4.4747e+03; mvc[14] = 1.5026e+03; mvc[15] = 5.1574e+03;


	// duration of the time window (in seconds) for the acquisition
	double twd = 0.05;

	// number of samples of the time window for the acquisition
	int nb_samples = (int)(twd*fs);

	std::cout << "Creating filters...\n";

	// High pass frequency
	double	cof_h = 50;

	// Low pass frequency
	double cof_l = 450;

	/*---- Create a Butterworth bandpass filter of 7nth order ----*/

	// order of the filter
	int filt_order = 7;

	double W_hp = 2 * cof_h / fs;
	double W_lp = 2 * cof_l / fs;

	double FrequencyBands[2] = { W_hp, W_lp };

	//Create the variables for the numerator and denominator coefficients
	double *Atmp = 0;				// temporary denominator
	double *Btmp = 0;				// temporary numerator
	std::vector<double> A(2 * filt_order + 1, 0);			// denominator
	std::vector<double> B(2 * filt_order + 1, 0);			// numerator

	Atmp = ComputeDenCoeffs(filt_order, FrequencyBands[0], FrequencyBands[1]);

	for (int k = 0; k<2 * filt_order + 1; k++)
	{
		A[k] = Atmp[k];

	}



	Btmp = ComputeNumCoeffs(filt_order, FrequencyBands[0], FrequencyBands[1], Atmp);

	for (int k = 0; k<2 * filt_order + 1; k++)
	{
		B[k] = Btmp[k];

	}


	/*---- Create a Butterworth lowpass filter of 7nth order ----*/


	// set the coefficients of the low-pass filter

	std::vector<double> A_low((int)filt_order + 1, 0);
	std::vector<double> B_low((int)filt_order + 1, 0);
	A_low[0] = 1.0; A_low[1] = -6.623530491642210; A_low[2] = 18.811576396477886; A_low[3] = -29.696521815511225; A_low[4] = 28.141621407266257; A_low[5] = -16.008501299509547; A_low[6] = 5.061527982504820; A_low[7] = -0.686172155496601;
	B_low[0] = 0.018819823477401e-08; B_low[1] = 0.131738764341804e-08; B_low[2] = 0.395216293025413e-08; B_low[3] = 0.658693821709022e-08; B_low[4] = 0.658693821709022e-08; B_low[5] = 0.395216293025413e-08; B_low[6] = 0.131738764341804e-08; B_low[7] = 0.018819823477401e-08;

	//std::vector<std::thread> thds;





	std::cout << "Setting up the DAQ board...\n";


	// setting up the DAQ board for acquisition

	int32 error = 0;
	TaskHandle taskHandle = 0;		// the ID of the task. Should be the same to all the functions related to a task

	int32 read;						// The actual number of samples read from each channel.

	std::string channels;
	if (DIMS < 2){
		channels = "Dev1/ai0";
	}
	else{
		channels = "Dev1/ai0:" + std::to_string(DIMS - 1);
	}

	char errBuff[4096] = { '\0' };


	// DAQmx analog voltage channel and timing parameters

	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));


	// Set the task for recording voltage from the first 4 channels in RSE terminal configuration, with values between -1 V and 1 V.
	
	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, channels.c_str(), "", DAQmx_Val_RSE, -5.0, 5.0, DAQmx_Val_Volts, NULL));
	


	// Sets the task to acquire data on the rising edges of the Sample Clock , the rate to be fs, continuous acquisition, and to acqiore 2 samples per channel

	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", fs, DAQmx_Val_Rising, DAQmx_Val_ContSamps, nb_samples)); //DAQmx_Val_FiniteSamps,DAQmx_Val_ContSamps


	float64 *data2 = new float64[DIMS*nb_samples];		// a vector to store the acquired data, grouped by channel (DAQmx_Val_GroupByChannel)


	if (pub_ch == 'y'){
		std::cout << "Initalize TCP connection...\n";
		node.init();
	}


	char do_trial;// = 'y';

	while (true){
		std::cout << "Ready for recording data\nDo you want to record a trial? (y/n) ";
		std::cin >> do_trial;

		if (do_trial == 'y'){

			folderName = base + "\\trial" + std::to_string(trial+1);

			CreateDirectory(folderName.c_str(), NULL);
			CreateDirectory((folderName+"\\timewindows").c_str(), NULL);

			// DAQmx Start the task

			DAQmxErrChk(DAQmxStartTask(taskHandle));



			while (true){


				// Read multiple floating-point samples from a task that contains 4 analog input channels. read 2 samples per channel, 
				// wait 0.1 sec for the acquisition of all the samples and set the buffer to 8 [chanels x (samples per channel)

				DAQmxErrChk(DAQmxReadAnalogF64(taskHandle, nb_samples, 0.06, DAQmx_Val_GroupByChannel, data2, DIMS*nb_samples, &read, NULL));

				if (data2 != NULL){

					//thds.push_back(std::thread(pubTW, B, A, B_low, A_low, mvc, data2, nb_samples));
					//std::thread t1(pubTW, B, A, B_low, A_low, mvc, data2, nb_samples);
					std::thread(pubTW, B, A, B_low, A_low, mvc, data2, nb_samples).detach();

				}
				else{
					std::cout << "Unable to read from DAQ device\n";
				}


				// break if the key 'q' is hit. if yes stop the acquisition

				if (_kbhit()){
					if (_getch() == 'q') {
						//cnt = 0;
						
						
						//Sleep(10);
						break;
					}

				}


			}
			
			DAQmxStopTask(taskHandle);
			saveData(folderName+"\\alldata", "wholeSignal",preProcessTW(B, A, B_low, A_low, mvc, allData));
			for (int i = 0; i < DIMS; i++){
				allData[i].clear();
				twData[i].clear();
			}
			cnt = 0;
			std::cout << "trial " << trial+1 << " saved\n";
			trial++;
		}
		else {
			if (do_trial == 'n'){
				std::cout << "ok\n";
				break;
			}
			else{
				std::cout << "Wrong choice\n";
			}
		}
	}




	// Stop and clear task 
Error:


	if (DAQmxFailed(error))

		DAQmxGetExtendedErrorInfo(errBuff, 2048);

	if (taskHandle != 0)  {

		DAQmxStopTask(taskHandle);

		DAQmxClearTask(taskHandle);

	}

	if (DAQmxFailed(error))

		printf("DAQmx Error: %s\n", errBuff);

	// memory deallocation
	if (data2 != NULL){
		delete data2;
	}

	std::cout << "Press a key to terminate the program\n";
	_getch();
	node.shutdownConnection();

	return 0;
}
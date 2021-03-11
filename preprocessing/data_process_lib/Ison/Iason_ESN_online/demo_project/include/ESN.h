/* ESN class
 *
 * Iason Batzianoulis
 *
 * describtion:
 *
 *
 *
 */

#include <iostream>
#include <string.h>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>



class ESN
{
	int nforgetpoints;
	int nInternalUnits;
	int nInputs;
	int nOutputs;
	std::vector< std::vector<double> > internalWeightsUnitSR;//(nInternalUnits, std::vector<float>(nInternalUnits));
	int nTotalUnits;
	int col_outWeights;

	std::vector< std::vector<double> > inputWeights;//(nInternalUnits,std::vector<float>(nInputs));
	std::vector< std::vector<double> > outputWeights;//(nOutputs,std::vector<float>(col_outWeights));
	std::vector< std::vector<double> > feedbackWeights;//(nInternalUnits, std::vector<float>(nOutputs));
	std::vector<double> inputScaling;//(nInputs);
	std::vector<double> inputShift;//(nInputs);
	std::vector<double>  teacherScaling;//(nOutputs);
	std::vector<double>  teacherShift;//(nOutputs);

	std::vector<double> feedbackScaling;//(nOutputs);
	float leakage;
	std::vector<double> timeConstants;//(nInternalUnits);
	std::vector< std::vector<double> > internalWeights;//(nInternalUnits,std::vector<float>(nInternalUnits));

public:
    ESN(void);
    ESN(int esn_nb);
    int setinterntalWUSR(std::ifstream& f, int k, int l);

    int setinputWeights(std::ifstream& f, int k, int l); //
    int setoutputWeights(std::ifstream& f, int k, int l); //
    int setfeedbackWeights(std::ifstream& f, int k, int l); //
    int setinputScaling(std::ifstream& f, int k); //
    int setinputShift(std::ifstream& f, int k); //
    int setteacherScaling(std::ifstream& f, int k); //
    int setteacherShift(std::ifstream& f, int k); //

    int setfeedbackScaling(std::ifstream& f, int k); //
    int settimeConstants(std::ifstream& f, int k);
    int setinternalWeights(std::ifstream& f, int k, int l);
    int printall();

    int test_esn(double **EMGsignal, int nforgetpoints, int nDatapoints);

    std::vector< std::vector<double> > compute_statematrix(double **EMGsignal, int nforgetpoints, int nDatapoints);
    std::vector<double> plain_esn(std::vector<double> tState);
    int s_classify(std::vector<std::vector<double> > outputSeq, int npoints);

    ~ESN(void);

};


ESN::ESN(void)
{
    nInternalUnits=1;

    std::cout<<"esn created\n";
}


ESN::~ESN(void)
{
//    std::cout<<"size: "<<sizeof(internalWeightsUnitSR)/(nInternalUnits*nInternalUnits)<<"\n";
//    std::cout<<"size2: "<<nInternalUnits<<"\n";

//    for (int i = 0; i < nInternalUnits; i++){
//        if(internalWeightsUnitSR[i]) delete[] internalWeightsUnitSR[i];
//        //std::cout<<"ok1 "<<i<<"\n";
//    }

//    if(internalWeightsUnitSR) delete[] internalWeightsUnitSR;


//    //delete[] internalWeightsUnitSR;
//    for (int i = 0; i < nInternalUnits; i++){
//        if(inputWeights[i]) delete[] inputWeights[i];
//	}
//    //delete inputWeights;
//	for (int i = 0; i < nOutputs; i++){
//        if(outputWeights[i]) delete[] outputWeights[i];
//	}
//    //delete outputWeights;
//	for (int i = 0; i < nInternalUnits; i++){
//        if(feedbackWeights[i]) delete[] feedbackWeights[i];
//	}
//    //delete feedbackWeights;

//    if(inputScaling) delete[] inputScaling;
//    if(inputShift) delete[] inputShift;
//    if(teacherScaling) delete[] teacherScaling;
//    if(teacherShift) delete[] teacherShift;

//    if(feedbackScaling) delete[] feedbackScaling;
//    if(timeConstants) delete[] timeConstants;

//	for (int i = 0; i < nInternalUnits; i++){
//        if(internalWeights[i]) delete[] internalWeights[i];
//	}
//    //delete internalWeights;

    //std::cout << "esn deleted\n";
    //cin.get();
}

ESN::ESN(int esn_nb){

   
    std::string file="data\\networks\\ESN";
    std::ostringstream convert;
    convert<<file<<esn_nb<<".txt";

    std::string nfile=convert.str();
    std::ifstream myfile(nfile.c_str());
    //std::cout<<nfile.c_str()<<"\n";

    if(myfile.is_open()){


        myfile>>nforgetpoints;
        myfile>>nInternalUnits;
        myfile>>nInputs;
        myfile>>nOutputs;
        col_outWeights=nInternalUnits+nInputs;

        setinterntalWUSR(myfile, nInternalUnits, nInternalUnits);
        
        myfile>>nTotalUnits;

        setinputWeights(myfile, nInternalUnits, nInputs);
        setoutputWeights(myfile, nOutputs, (nInternalUnits+nInputs));
        setfeedbackWeights(myfile, nInternalUnits, nOutputs);
        setinputScaling(myfile, nInputs);
        setinputShift(myfile, nInputs);
        setteacherScaling(myfile, nOutputs);
        setteacherShift(myfile, nOutputs);

        setfeedbackScaling(myfile, nOutputs);
        settimeConstants(myfile,nInternalUnits);
        myfile>>leakage;
        setinternalWeights(myfile,nInternalUnits, nInternalUnits);


       
        myfile.close();


    }
    else std::cout<<"Unable to open file"<<"\n";


}

int ESN::setinterntalWUSR(std::ifstream& f, int k, int l){
   
    internalWeightsUnitSR.resize(k,std::vector<double>());
    

    for(int i=0;i<k;i++){
       
        double tmp=0;
        for(int j=0;j<l;j++){
           
            f>>tmp;
            internalWeightsUnitSR[i].push_back(tmp);
          
        }
    }

    return 0;
}


int ESN::setinputWeights(std::ifstream& f, int k, int l){
    
    inputWeights.resize(k,std::vector<double>());
    
    double tmp=0;
    for(int i=0;i<k;i++){
      
        for(int j=0;j<l;j++){
           
            f>>tmp;
            inputWeights[i].push_back(tmp);
        }
    }
  
    return 0;
}


int ESN::setoutputWeights(std::ifstream& f, int k, int l){
   
    outputWeights.resize(k,std::vector<double>());

    double tmp=0;
    for(int i=0;i<k;i++){
   
        for(int j=0;j<l;j++){
            
            f>>tmp;
            outputWeights[i].push_back(tmp);
        }
    }
  
    return 0;
}

int ESN::setfeedbackWeights(std::ifstream& f, int k, int l){
   
    feedbackWeights.resize(k,std::vector<double>());
   
    double tmp=0;
    for(int i=0;i<k;i++){
      
        for(int j=0;j<l;j++){
        
            f>>tmp;
            feedbackWeights[i].push_back(tmp);//[j];
        }
    }
  
    return 0;
}

int ESN::setinputScaling(std::ifstream& f, int k){
   
    double tmp=0;
    for(int i=0;i<k;i++) {
        f>>tmp;
        inputScaling.push_back(tmp);
    }

    return 0;
}

int ESN::setinputShift(std::ifstream& f, int k){

    double tmp=0;
    for(int i=0;i<k;i++){
        f>>tmp;
        inputShift.push_back(tmp);
    }
   
    return 0;
}

int ESN::setteacherScaling(std::ifstream& f, int k){
  
    double tmp=0;
    for(int i=0;i<k;i++){
        f>>tmp;
        teacherScaling.push_back(tmp);
    }

    return 0;
}

int ESN::setteacherShift(std::ifstream& f, int k){
   
    double tmp=0;
    for(int i=0;i<k;i++){
        f>>tmp;
        teacherShift.push_back(tmp);
    }
   
    return 0;
}

int ESN::setfeedbackScaling(std::ifstream& f, int k){
   
    double tmp=0;
    for(int i=0;i<k;i++){
        f>>tmp;
        feedbackScaling.push_back(tmp);
    }
    
    return 0;
}

int ESN::settimeConstants(std::ifstream& f, int k){
   
    double tmp=0;
    for(int i=0;i<k;i++){
        f>>tmp;
        timeConstants.push_back(tmp);
    }
  
    return 0;
}

int ESN::setinternalWeights(std::ifstream& f, int k, int l){
   
    internalWeights.resize(k,std::vector<double>());
   
    for(int i=0;i<k;i++){
       
        double tmp=0;
        for(int j=0;j<l;j++){
            
            f>>tmp;
            internalWeights[i].push_back(tmp);
        }
    }
   
    return 0;
}

int ESN::printall(){
    std::cout<<"number of Internal Units: "<<nInternalUnits<<std::endl;
    std::cout<<"number of Input Units: "<<nInputs<<std::endl;
    std::cout<<"number of Output Units: "<<nOutputs<<std::endl;


    std::cout<<"internal Weights UnitSR"<<std::endl;

    for(int i=0;i<nInternalUnits;i++){
        for(int j=0;j<nInternalUnits;j++) std::cout<<internalWeightsUnitSR[i][j]<<" ";
        std::cout<<std::endl;
    }


    std::cout<<"number of Total Units: "<<nTotalUnits<<std::endl;
 
    std::cout<<"input Weights:"<<std::endl;
    for(int i=0;i<nInternalUnits;i++){
        for(int j=0;j<nInputs;j++) std::cout<<inputWeights[i][j]<<" ";
        std::cout<<std::endl;
    }
   
    std::cout<<"output Weights:"<<std::endl;
    for(int i=0;i<nOutputs;i++){
        for(int j=0;j<(nInputs+nInternalUnits);j++) std::cout<<outputWeights[i][j]<<" ";
        std::cout<<std::endl;
    }
    
    std::cout<<"feedback Weights:"<<std::endl;
    for(int i=0;i<nInternalUnits;i++){
        for(int j=0;j<nOutputs;j++) std::cout<<feedbackWeights[i][j]<<" ";
        std::cout<<std::endl;
    }
   
    std::cout<<"input Scalling:"<<std::endl;
    for(int i=0;i<nInputs;i++) std::cout<<inputScaling[i]<<" ";
    std::cout<<std::endl;
  
    std::cout<<"input Shift:"<<std::endl;
    for(int i=0;i<nInputs;i++) std::cout<<inputShift[i]<<" ";
    std::cout<<std::endl;
 
    std::cout<<"teacher Scaling:"<<std::endl;
    for(int i=0;i<nOutputs;i++) std::cout<<teacherScaling[i]<<" ";
    std::cout<<std::endl;

	std::cout<<"teacher Shift:"<<std::endl;
    for(int i=0;i<nOutputs;i++) std::cout<<teacherShift[i]<<" ";
    std::cout<<std::endl;
  
    std::cout<<"feedback Scaling:"<<std::endl;
    for(int i=0;i<nOutputs;i++) std::cout<<feedbackScaling[i]<<" ";
    std::cout<<std::endl;
 
    std::cout<<"time Constants:"<<std::endl;
    for(int i=0;i<nInternalUnits;i++) std::cout<<timeConstants[i]<<" ";
    std::cout<<std::endl;

    std::cout<<"internal Weights:"<<std::endl;
    for(int i=0;i<nInternalUnits;i++){
        for(int j=0;j<nInternalUnits;j++) std::cout<<internalWeights[i][j]<<" ";
        std::cout<<std::endl;
    }
   
    std::cout<<"leakage: "<<leakage<<std::endl;


    return 0;
}

int ESN::test_esn(double** EMGsignal, int nforgetpoints, int nDatapoints){


    std::vector< std::vector<double> > stateCollection=compute_statematrix(EMGsignal,nforgetpoints,nDatapoints);

	//std::cout << "size state: " << stateCollection.size() << "," << stateCollection[0].size() << "\n";


    std::vector< std::vector<double> > outputSequence(nDatapoints-nforgetpoints,std::vector<double>(nOutputs));
    for(int i=0;i<nDatapoints-nforgetpoints;i++){
       
        for(int j=0;j<nOutputs;j++){
            double tmp2=0;
            for(int k=0;k<nInputs+nInternalUnits;k++){
                tmp2=stateCollection[i][k]*outputWeights[j][k]+tmp2;
               

            }
           

            outputSequence[i][j]=tmp2;
        }

        
    }

    std::vector< std::vector<double> > repmat(nDatapoints-nforgetpoints,std::vector<double>(nOutputs));//new double*[nDatapoints-nforgetpoints];
    for(int i=0;i<(nDatapoints-nforgetpoints);i++){
       
        for(int j=0;j<nOutputs;j++) repmat[i][j]=teacherShift[j];


    }
   
    for(int i=0;i<(nDatapoints-nforgetpoints);i++){
        for(int j=0;j<nOutputs;j++){
            outputSequence[i][j]=outputSequence[i][j]-repmat[i][j];
            
        }
        
    }
    std::vector< std::vector<double> > invDiagTeacherScalling(nOutputs,std::vector<double>(nOutputs));//=new double*[nOutputs];
    for(int i=0;i<nOutputs;i++){
        
        for(int j=0;j<nOutputs;j++){
            invDiagTeacherScalling[i][j]=0;
            if(i==j){
                invDiagTeacherScalling[i][j]=1/teacherScaling[i];
            }
        }
    }



    for(int i=0;i<nDatapoints-nforgetpoints;i++){
        for(int j=0;j<nOutputs;j++){
            double tmp2=0;
            for(int k=0;k<nOutputs;k++){
                tmp2=tmp2+outputSequence[i][k]*invDiagTeacherScalling[k][j];
            }
            outputSequence[i][j]=tmp2;
        }
    }


    int classification_result=s_classify(outputSequence,nDatapoints-nforgetpoints);


    return classification_result+1;
}

std::vector<std::vector<double> > ESN::compute_statematrix(double **EMGsignal, int nforgetpoints, int nDatapoints){
   
    std::vector<double> totalstate(nInternalUnits+nInputs+nOutputs);
    for(int i=0;i<nInternalUnits+nInputs+nOutputs;i++) totalstate[i]=0;

    std::vector<std::vector<double> > stateCollectMat(nDatapoints-nforgetpoints);//= new double*[nDatapoints-nforgetpoints];
    std::vector<double> netOut(nOutputs);
    std::vector<double> in(nInputs);
    std::vector<double> internalState;
    std::vector<double> tin(nInternalUnits + nInputs);
    int collectdata=0;

    std::vector<double> tmp(nInternalUnits+nInputs,0);

    for(int i=0;i<nDatapoints;i++){

        for(int j=0;j<nInternalUnits+nInputs;j++) tmp[j]=0;
  
        for(int j=0;j<nInputs;j++){
            in[j]=inputScaling[j]*EMGsignal[i][j]+inputShift[j];
            
        }

      

        for(int j=nInternalUnits;j<nInternalUnits+nInputs;j++){
            totalstate[j]=in[j-nInternalUnits];
        }



        internalState=plain_esn(totalstate);

        for(int j=0;j<nInternalUnits+nInputs;j++){
            if(j<nInternalUnits) {
                tin[j]=internalState[j];
            }else {
                tin[j]=in[j-nInternalUnits];
            }
        }



        for(int j=0;j<nOutputs;j++){
            double tnumber=0;
            for(int k=0;k<nInternalUnits+nInputs;k++){
                tnumber=tnumber+outputWeights[j][k]*tin[k];
            }
            netOut[j]=tnumber;
        }

        for(int j=0;j<nInternalUnits;j++) totalstate[j]=internalState[j];

        for(int j=nInternalUnits+nInputs;j<nInternalUnits+nInputs+nOutputs;j++) totalstate[j]=netOut[j-(nInternalUnits+nInputs)];

 




        for(int j=0;j<nInternalUnits+nInputs;j++) {tmp[j]=totalstate[j];}


        if(i>nforgetpoints-1){
          
            stateCollectMat[collectdata]=tmp;
            collectdata=collectdata+1;
            
        }

    }

    return stateCollectMat;
}

std::vector<double> ESN::plain_esn(std::vector<double> tState){

    std::vector<std::vector<double> > tmp(nInternalUnits,std::vector<double>(nInternalUnits+nInputs+nOutputs));//=new double*[nInternalUnits];
    std::vector<std::vector<double> > diagfeedback(nOutputs, std::vector<double>(nOutputs));//=new double*[nOutputs];

    for(int i=0;i<nOutputs;i++){

       
        for(int j=0;j<nOutputs;j++){

            diagfeedback[i][j]=0;
            if(j==i) diagfeedback[i][j]=feedbackScaling[i];

        }
    }




    std::vector<std::vector<double> > feedS(nInternalUnits,std::vector<double>(nOutputs));
    for(int i=0;i<nInternalUnits;i++){
       
        for(int j=0;j<nOutputs;j++){
            double tmp2=0;
            for(int k=0;k<nOutputs;k++){
                tmp2=tmp2+feedbackWeights[i][k]*diagfeedback[k][j];
            }
            feedS[i][j]=tmp2;
        }
    }

   
    for(int i=0;i<nInternalUnits;i++){
        
        for(int j=0;j<nInternalUnits+nInputs+nOutputs;j++){
            //  std::cout<<j<<" ";
            if(j<nInternalUnits) tmp[i][j]=internalWeights[i][j];
            if((j>=nInternalUnits)&&(j<nInternalUnits+nInputs)) tmp[i][j]=inputWeights[i][j-nInternalUnits];
            if(j>=nInternalUnits+nInputs) tmp[i][j]=feedS[i][j-(nInternalUnits+nInputs)];
        }
    }
   

    std::vector<double> internalState(nInternalUnits);
    for(int i=0;i<nInternalUnits;i++){
        double tmp2=0;
        for(int j=0;j<nInternalUnits+nInputs+nOutputs;j++){
            tmp2=tmp2+tmp[i][j]*tState[j];
        }
        internalState[i]=tanh(tmp2);
    }


    return internalState;
}

int ESN::s_classify(std::vector<std::vector<double> > outputSeq, int npoints){

    double *classes=new double[nOutputs];
    for(int i=0;i<nOutputs;i++){
        classes[i]=0;
    }
    
    for(int i=0;i<npoints;i++){
        for(int j=0;j<nOutputs;j++){
        
            classes[j]=classes[j]+outputSeq[i][j];
        }
       
    }
    double c_max=-100;
    int index=-100;
    for(int i=0;i<nOutputs;i++){

        if(classes[i]>c_max){
            c_max=classes[i];
            index=i;
        }

    }

    if(classes) delete[] classes;
    return index;
}






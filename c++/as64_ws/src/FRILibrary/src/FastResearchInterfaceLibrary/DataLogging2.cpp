//  ---------------------- Doxygen info ----------------------
//! \file DataLogging.cpp
//!
//! \brief
//! Implementation file for the class DataLogging
//!
//! \details
//! The class DataLogging provides the possibility of writing all important
//! data that is exchanged between the remote host and the KRC unit to an
//! output file. For further details, please refer to the file
//! DataLogging.h
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! This file is part of the Fast Research Interface Library.
//! \n\n
//! The Fast Research Interface Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Fast Research Interface Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Fast Research Interface Library. If not, see
//! http://www.gnu.org/licenses.
//! \n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//!
//! \date October 2013
//!
//! \version 1.0.1
//!
//!	\author Torsten Kroeger, tkr@stanford.edu
//!
//!
//! \note Copyright (C) 2013 Stanford University.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <DataLogging2.h>
#include <FastResearchInterface.h>
#include <InitializationFileEntry.h>
#include <algorithm>
#include <OSAbstraction.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <string.h>

#define NUMBER_OF_ELEMENTS_PER_ENTRY	58
#define OUTPUT_FILE_STRING_LENGTH		1024
#define TIME_STRING_LENGTH				128

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif

#define FIELD_DATA_TYPE_GROUPS_COUNT	7
#define MAX_FIELDS_GROUP_MEMBERS_COUNT	20

// ****************************************************************
// Constructor
//
DataLogging2::DataLogging2(	const char				*RobotName
		,	const char				*LoggingPath
		,	const char				*LoggingFileName
		,	const unsigned int	&MaxNumberOfEntries
		,	const char				*LoggingStrategy)
:DataLogging(RobotName, LoggingPath, LoggingFileName, MaxNumberOfEntries)
{
	this->LoggingStrategy 		= (char*)LoggingStrategy;

	if (stricmp((const char*)this->LoggingStrategy, "Standard")){
		const char* tmpFields[] = {
				//----------------------------------------------------------------------
				//Category			Field name						Field label
				//----------------------------------------------------------------------

				//SINGLE VALUED FIELDS
				"Msr-Head", 		"SequenceCount",				"msr.sendSeqCount",
				"Msr-Head", 		"ReflectedSequenceCount",		"msr.reflSeqCount",
				"Msr-Head", 		"PacketSize",					"msr.packetSize",
				"Msr-Head",			"DatagramId",					"msr.datagramId",
				"Msr-Interface",	"Timestamp",					"msr.timestamp",
				"Msr-Interface", 	"State",						"msr.state",
				"Msr-Interface", 	"Quality",						"msr.quality",
				"Msr-Interface", 	"MsrSampleTime",				"msr.desiredMsrSampleTime",
				"Msr-Interface", 	"CmdSampleTime",				"msr.desiredCmdSampleTime",
				"Msr-Interface", 	"SafetyLimits",					"msr.safetyLimits",
				"Msr-Interface", 	"CommAnswerRate",				"msr.stat.answerRate",
				"Msr-Interface", 	"CommLatency",					"msr.stat.latency",
				"Msr-Interface", 	"CommJitter",					"msr.stat.jitter",
				"Msr-Interface", 	"CommMissRate",					"msr.stat.missRate",
				"Msr-Interface", 	"CommMissCounter",				"msr.stat.missCounter",
				"Msr-Robot", 		"Power", 						"msr.power",
				"Msr-Robot", 		"ControlStrategy",				"msr.control",
				"Msr-Robot", 		"DriveError",					"msr.error",
				"Msr-Robot", 		"DriveWarning",					"msr.warning",
				"Cmd-Head", 		"SequenceCount",				"cmd.sendSeqCount",
				"Cmd-Head", 		"ReflectedSequenceCount",		"cmd.reflSeqCount",
				"Cmd-Head", 		"PacketSize",					"cmd.packetSize",
				"Cmd-Head", 		"DatagramId",					"cmd.datagramId",
				"Cmd-Command", 		"RelevantCommandedFlags",		"cmd.cmdFlags",

				//SIZE_USER_DATA SIZED ATTRIBUTES
				"Msr-KRL",			"RealData",						"msr.realData",
				"Msr-KRL",			"IntegerData",					"msr.intData",
				"Msr-KRL",			"BooleanData",					"msr.boolData",
				"Cmd-KRL",			"RealData",						"cmd.realData",
				"Cmd-KRL",			"IntegerData",					"cmd.intData",
				"Cmd-KRL",			"BooleanData",					"cmd.boolData",

				//NUMBER_OF_JOINTS SIZED ATTRIBUTES
				"Msr-Robot", 		"DriveTemperature",				"msr.temperature",
				"Msr-Data",			"MeasuredJointPosition",		"msr.msrJntPos",
				"Msr-Data",			"CommandedJointPosition",		"msr.cmdJntPos",
				"Msr-Data",			"CommandedJointOffset",			"msr.cmdJntPosFriOffset",
				"Msr-Data",			"MeasuredJointTorque",			"msr.msrJntTrq",
				"Msr-Data",			"EstimatedExternalTorque",		"msr.estExtJntTrq",
				"Msr-Data",			"GravityVector",				"msr.gravity",
				"Cmd-Command", 		"JointPosition",				"cmd.jntPos",
				"Cmd-Command", 		"AdditionalTorque",				"cmd.addJntTrq",
				"Cmd-Command", 		"JointStiffness",				"cmd.jntStiffness",
				"Cmd-Command", 		"JointDamping",					"cmd.jntDamping",

				//NUMBER_OF_FRAME_ELEMENTS SIZED ATTRIBUTES
				"Msr-Data",			"MeasuredCartesianPosition",	"msr.msrCartPos",
				"Msr-Data",			"CommandedCartesianPosition",	"msr.cmdCartPos",
				"Msr-Data",			"CommandedCartesianOffset",		"msr.cmdCartPosFriOffset",
				"Cmd-Command", 		"CartesianPosition",			"cmd.cartPos",

				//NUMBER_OF_CART_DOFS SIZED ATTRIBUTES
				"Msr-Data",			"EstimatedTCPForce",			"msr.estExtTcpFT",
				"Cmd-Command", 		"AdditionalTCPForce",			"cmd.addTcpFT",
				"Cmd-Command", 		"CartesianStiffness",			"cmd.cartStiffness",
				"Cmd-Command", 		"CartesianDamping",				"cmd.cartDamping",

				//NUMBER_OF_CART_DOFS*NUMBER_OF_JOINTS SIZED ATTRIBUTES
				"Msr-Data",			"JacobianMatrix",				"msr.jacobian",

				//NUMBER_OF_JOINTS*NUMBER_OF_JOINTS SIZED ATTRIBUTES
				"Msr-Data",			"MassMatrix",					"msr.massMatrix",

				//NOT FOUND!!
				"",					"SequenceCount",				"n/a"
		};

		const int tmpFieldLengths[] = {
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				1,
				SIZE_USER_DATA,
				SIZE_USER_DATA,
				SIZE_USER_DATA,
				SIZE_USER_DATA,
				SIZE_USER_DATA,
				SIZE_USER_DATA,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS,
				NUMBER_OF_FRAME_ELEMENTS,
				NUMBER_OF_FRAME_ELEMENTS,
				NUMBER_OF_FRAME_ELEMENTS,
				NUMBER_OF_FRAME_ELEMENTS,
				NUMBER_OF_CART_DOFS,
				NUMBER_OF_CART_DOFS,
				NUMBER_OF_CART_DOFS,
				NUMBER_OF_CART_DOFS,
				NUMBER_OF_CART_DOFS*NUMBER_OF_JOINTS,
				NUMBER_OF_JOINTS*NUMBER_OF_JOINTS,
				0
		};

		for (int i=0; i< 3 * (AVAILABLE_READINGS_COUNT+1); i++){
			strcpy(this->Fields[i], tmpFields[i]);
		}

		for (int i=0; i< AVAILABLE_READINGS_COUNT+1; i++){
			this->FieldLengths[i] = tmpFieldLengths[i];
		}

		this->FieldGroupsCounts 	= new unsigned int[FIELD_DATA_TYPE_GROUPS_COUNT];
		memset(this->FieldGroupsCounts, 0x0,
				FIELD_DATA_TYPE_GROUPS_COUNT * sizeof(unsigned int));

		this->FieldGroupsIndexes 	= new unsigned int*[FIELD_DATA_TYPE_GROUPS_COUNT];
		for (int i = 0; i < FIELD_DATA_TYPE_GROUPS_COUNT; i++)
		{
			this->FieldGroupsIndexes[i] 	= 	new unsigned int[MAX_FIELDS_GROUP_MEMBERS_COUNT];
			memset(		this->FieldGroupsIndexes[i]
			       		                         ,	0x0
			       		                         ,	MAX_FIELDS_GROUP_MEMBERS_COUNT * sizeof(unsigned int)	);
		}

		for (int i = 0; i < NUMBER_OF_ELEMENTS_PER_ENTRY; i++)
			delete[]	LoggingMemory[i];
		delete[]	LoggingMemory;

		this->TotalFieldsCount = this->ParseFields(this->FieldGroupsIndexes, this->FieldGroupsCounts);

#undef NUMBER_OF_ELEMENTS_PER_ENTRY
#define NUMBER_OF_ELEMENTS_PER_ENTRY this->TotalFieldsCount

		this->LoggingMemory = new float*[NUMBER_OF_ELEMENTS_PER_ENTRY];
		for (int i = 0; i < NUMBER_OF_ELEMENTS_PER_ENTRY; i++)
		{
			this->LoggingMemory[i]	= new float[this->MaximumNumberOfEntries];
			memset(		this->LoggingMemory[i]
	                  ,	0x0
	                  , this->MaximumNumberOfEntries * sizeof(float));
		}
	}
}


// ****************************************************************
// Destructor
//
DataLogging2::~DataLogging2(void)
{
	for (int i = 0; i < FIELD_DATA_TYPE_GROUPS_COUNT; i++)
		delete[] this->FieldGroupsIndexes[i];

	delete[] this->FieldGroupsIndexes;
	delete[] this->FieldGroupsCounts;

	delete[] this->Fields;
	delete[] this->FieldLengths;
	delete[] this->FieldGroupsIndexes;
	delete[] this->FieldGroupsCounts;
	delete[] this->LoggingStrategy;
}

// ****************************************************************
// PrepareLogging()
//
int DataLogging2::PrepareLogging(	const unsigned int	&ControlScheme
		,	const char				*FileIdentifier)
{
	fprintf(stdout,	"Prepare started...\n");
	char				TimeString[TIME_STRING_LENGTH];

	unsigned int		i		=	0;

	time_t				CurrentDayTime;

	memset(		TimeString
			,	0x0
			,	TIME_STRING_LENGTH * sizeof(char));

	this->CurrentControlScheme	=	ControlScheme;

	if (this->CurrentObjectState	==	DataLogging::PrepareLoggingCalled)
	{
		this->WriteToFile();
	}

	GetSystemTimeInSeconds(true);

	fprintf(stdout,	"Intermediate point...\n");


	//REMOVE

	//------------------------

#ifdef _NTO_

	struct _clockperiod 	ClockResolution;

	ClockResolution.nsec = 10000;	//ns
	ClockResolution.fract = 0;

	ClockPeriod(CLOCK_REALTIME, &ClockResolution, NULL, 0);
	//------------------------

#endif

	memset(		this->CompleteOutputFileString
			,	0x0
			,	OUTPUT_FILE_STRING_LENGTH * sizeof(char));

	for (i = 0; i < NUMBER_OF_ELEMENTS_PER_ENTRY; i++)
	{
		memset(		this->LoggingMemory[i]
		       		                    ,	0x0
		       		                    ,	this->MaximumNumberOfEntries * sizeof(float)	);
	}

	CurrentDayTime = time(NULL);
	strftime(TimeString, TIME_STRING_LENGTH, "%y%m%d-%H%M%S", localtime(&CurrentDayTime));
	if (FileIdentifier == NULL)
	{
		sprintf(this->CompleteOutputFileString, "%s%s-%s-%s", this->OutputPath, TimeString, this->MachineName, this->OutputFileName);
	}
	else
	{
		sprintf(this->CompleteOutputFileString, "%s%s-%s-%s-%s", this->OutputPath, TimeString, FileIdentifier, this->MachineName, this->OutputFileName);
	}

	if ( (this->OutputFileHandler = fopen(this->CompleteOutputFileString, "w") ) == NULL)
	{
		return(EBADF);
	}
	else
	{
		fprintf(this->OutputFileHandler, "Logging file of the KUKA Fast Research Interface: %s\n", this->CompleteOutputFileString);
		fprintf(this->OutputFileHandler, "This file contains all important control values and importable to Matlab and MS Excel.\n");
		fprintf(this->OutputFileHandler, "%s\n", ctime( &CurrentDayTime));
		fprintf(this->OutputFileHandler, "Robot name: %s\n", this->MachineName);

		switch (this->CurrentControlScheme)
		{
		case FastResearchInterface::JOINT_POSITION_CONTROL:
			fprintf(this->OutputFileHandler, "Active control scheme: joint position control\n\n");
			if (!stricmp((const char*)this->LoggingStrategy, "Standard"))
				fprintf(this->OutputFileHandler, "Counter	KRCTime	LocalTime	ActFJ1	ActFJ2	ActFJ3	ActFJ4	ActFJ5	ActFJ6	ActFJ7	UDesJ1	UDesJ2	UDesJ3	UDesJ4	UDesJ5	UDesJ6	UDesJ7	ActJ1	ActJ2	ActJ3	ActJ4	ActJ5	ActJ6	ActJ7	KDesJ1	KDesJ2	KDesJ3	KDesJ4	KDesJ5	KDesJ6	KDesJ7\n");
			break;
		case FastResearchInterface::CART_IMPEDANCE_CONTROL:
			fprintf(this->OutputFileHandler, "Active control scheme: Cartesian impedance control\n\n");
			if (!stricmp((const char*)this->LoggingStrategy, "Standard"))
				fprintf(this->OutputFileHandler, "Counter	KRCTime	LocalTime	DesKx	DesKy	DesKz	DesKa	DesKb	DesKc	DesDx	DesDy	DesDz	DesDa	DesDb	DesDc	UDesFx	UDesFy	UDesFz	UDesFa	UDesFb	UDesFc\n");
			break;
		case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
			fprintf(this->OutputFileHandler, "Active control scheme: joint impedance control\n\n");
			if (!stricmp((const char*)this->LoggingStrategy, "Standard"))
				fprintf(this->OutputFileHandler, "Counter	KRCTime	LocalTime	ActFJ1	ActFJ2	ActFJ3	ActFJ4	ActFJ5	ActFJ6	ActFJ7	UDesPJ1	UDesPJ2	UDesPJ3	UDesPJ4	UDesPJ5	UDesPJ6	UDesPJ7	ActPJ1	ActPJ2	ActPJ3	ActPJ4	ActPJ5	ActPJ6	ActPJ7	KDesPJ1	KDesPJ2	KDesPJ3	KDesPJ4	KDesPJ5	KDesPJ6	KDesPJ7	DesKJ1	DesKJ2	DesKJ3	DesKJ4	DesKJ5	DesKJ6	DesKJ7	DesDJ1	DesDJ2	DesDJ3	DesDJ4	DesDJ5	DesDJ6	DesDJ7	UDesFJ1	UDesFJ2	UDesFJ3	UDesFJ4	UDesFJ5	UDesFJ6	UDesFJ7	KOffPJ1	KOffPJ2	KOffPJ3	KOffPJ4	KOffPJ5	KOffPJ6	KOffPJ7\n");
			break;
		case FastResearchInterface::JOINT_TORQUE_CONTROL:
			fprintf(this->OutputFileHandler, "Active control scheme: joint torque control\n\n");
			if (!stricmp((const char*)this->LoggingStrategy, "Standard"))
				fprintf(this->OutputFileHandler, "Counter	KRCTime	LocalTime	ActFJ1	ActFJ2	ActFJ3	ActFJ4	ActFJ5	ActFJ6	ActFJ7	ActPJ1	ActPJ2	ActPJ3	ActPJ4	ActPJ5	ActPJ6	ActPJ7	UDesFJ1	UDesFJ2	UDesFJ3	UDesFJ4	UDesFJ5	UDesFJ6	UDesFJ7\n");
			break;
		default:
			return(EINVAL);
		}

		if (stricmp((const char*)this->LoggingStrategy, "Standard")){
			fprintf(stdout,	"Logging strategy headers written...\n");
			this->WriteFieldsDescription(this->OutputFileHandler);
		}
		else {
			fprintf(stdout,	"Standard headers written...\n");
		}
	}

	fflush(this->OutputFileHandler);

	this->CurrentObjectState	=	DataLogging::PrepareLoggingCalled;
	this->OutputCounter			=	0;

	return(EOK);
}


// ****************************************************************
// AddEntry()
//
void DataLogging2::AddEntry(		const FRIDataReceivedFromKRC		&ReceivedFRIData
								,	const FRIDataSendToKRC		&SentFRIData		)
{
	if (!stricmp((const char*)this->LoggingStrategy, "Standard"))
		this->DataLogging::AddEntry(ReceivedFRIData, SentFRIData);

	else {
		unsigned int	index	=	0;
		unsigned int	lmpos	= 	this->OutputCounter % this->MaximumNumberOfEntries;
		unsigned int	lmcpos	=	0;

		this->LoggingMemory[0][lmpos]	=	GetSystemTimeInSeconds();
		lmcpos = 1;

		for(int i=0; i < FIELD_DATA_TYPE_GROUPS_COUNT; i++){
			if (this->FieldGroupsCounts[i]>0)
				for (int j=0; j < this->FieldGroupsCounts[i]; j++){
					index = this->FieldGroupsIndexes[i][j];
					for (int k=0; k < this->FieldLengths[index]; k++){
						switch (index){
						case 0: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.Header.FRISequenceCounterForUDPPackages; break;
						case 1: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.Header.FRIReflectedSequenceCounterForUDPPackages; break;
						case 2: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.Header.FRIPackageSizeInBytes; break;
						case 3: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.Header.FRIDatagramID; break;
						case 4: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRITimeStamp; break;
						case 5: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRIState; break;
						case 6: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRIQuality; break;
						case 7: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRISampleTimePeriodForDataSentFromKRC; break;
						case 8: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRISampleTimePeriodForDataSentToKRC; break;
						case 9: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRICommunicationTimeLimitsForSafety; break;
						case 10: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRIStatistics.AverageRateOfAnsweredPackages; break;
						case 11: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRIStatistics.AverageLatencyInSeconds; break;
						case 12: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRIStatistics.AverageJitterInSeconds; break;
						case 13: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRIStatistics.AverageRateOfMissedPackages; break;
						case 14: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.InterfaceState.FRIStatistics.AbsoluteNumberOfMissedPackages; break;
						case 15: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.Robot.FRIRobotPower; break;
						case 16: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.Robot.FRIRobotControl; break;
						case 17: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.Robot.FRIDriveError; break;
						case 18: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.Robot.FRIDriveWarning; break;
						case 19: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.Header.FRISequenceCounterForUDPPackages; break;
						case 20: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.Header.FRIReflectedSequenceCounterForUDPPackages; break;
						case 21: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.Header.FRIPackageSizeInBytes; break;
						case 22: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.Header.FRIDatagramID; break;
						case 23: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.CommandValues.FRIRobotCommandDataFlags; break;
						case 24: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.SharedKRLVariables.FRIFloatingPointValuesInKRC[k]; break;
						case 25: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.SharedKRLVariables.FRIIntegerValuesInKRC[k]; break;
						//case 26: FieldDescription = "msr.boolData"; break;
						case 27: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.SharedKRLVariables.FRIFloatingPointValuesInKRC[k]; break;
						case 28: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.SharedKRLVariables.FRIIntegerValuesInKRC[k]; break;
						//case 29: FieldDescription = "cmd.boolData"; break;
						case 30: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.Robot.FRIDriveTemperature[k]; break;
						case 31: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRIMeasuredJointPositionVectorInRad[k]; break;
						case 32: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRICommandedJointPostionVectorFromKRC[k]; break;
						case 33: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRICommandedJointPostionOffsetVectorFromKRC[k]; break;
						case 34: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRIMeasuredJointTorqueVectorInNm[k]; break;
						case 35: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRIEstimatedExternalJointTorqueVectorInNm[k]; break;
						case 36: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRIGravityVectorInJointSpace[k]; break;
						case 37: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.CommandValues.FRICommandedJointPositionVectorInRad[k]; break;
						case 38: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.CommandValues.FRICommandedAdditionalJointTorqueVectorInNm[k]; break;
						case 39: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.CommandValues.FRICommandedJointStiffnessVectorInNmPerRad[k]; break;
						case 40: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.CommandValues.FRICommandedNormalizedJointDampingVector[k]; break;
						case 41: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRIMeasuredCartesianFrame[k]; break;
						case 42: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRICommandedCartesianFrameFromKRC[k]; break;
						case 43: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRICommandedCartesianFrameOffsetFromKRC[k]; break;
						case 44: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.CommandValues.FRICommandedCartesianFrame[k]; break;
						case 45: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRIEstimatedCartesianForcesAndTorques[k]; break;
						case 46: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.CommandValues.FRICommandedAdditionalCartesianForceTorqueVector[k]; break;
						case 47: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.CommandValues.FRICommandedCartesianStiffnessVector[k]; break;
						case 48: this->LoggingMemory[lmcpos][lmpos] = SentFRIData.CommandValues.FRICommandedNormalizedCartesianDampingVector[k]; break;
						case 49: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRIJacobianMatrix[k]; break;
						case 50: this->LoggingMemory[lmcpos][lmpos] = ReceivedFRIData.MeasuredData.FRIMassMatrix[k]; break;
						}
						lmcpos++;
					}
				}
		}
		this->OutputCounter++;
	}
	return;
}

// ****************************************************************
// WriteToFile()
//
int DataLogging2::WriteToFile(void)
{
	int					ReturnValue			=	0;

	unsigned int		StartIndex			=	0
					,	StopIndex			=	0
					,	Counter				=	0
					,	i					=	0
					,	ElementsPerLine		=	0;

	if (this->CurrentObjectState	!=	DataLogging::PrepareLoggingCalled)
	{
		return(EPERM);
	}
	else
	{
		this->CurrentObjectState	=	DataLogging::WriteToFileCalled;
	}

	//REMOVE
	//------------------------
#ifdef _NTO_
	struct _clockperiod 	ClockResolution;

	ClockResolution.nsec = 1000000;	//ns
	ClockResolution.fract = 0;

	ClockPeriod(CLOCK_REALTIME, &ClockResolution, NULL, 0);
#endif
	//------------------------

	if (this->OutputCounter > this->MaximumNumberOfEntries)
	{
		StartIndex	=	(this->OutputCounter + 1) % this->MaximumNumberOfEntries;
		StopIndex	=	StartIndex + this->MaximumNumberOfEntries - 1;
	}
	else
	{
		StartIndex	=	0;
		StopIndex	=	this->OutputCounter - 1;
	}

	if (!stricmp((const char*)this->LoggingStrategy, "Standard"))
		switch (this->CurrentControlScheme)
		{
		case FastResearchInterface::JOINT_POSITION_CONTROL:
			ElementsPerLine	=	2 + 4 * NUMBER_OF_JOINTS;
			break;
		case FastResearchInterface::CART_IMPEDANCE_CONTROL:
			ElementsPerLine	=	2 + 3 * NUMBER_OF_CART_DOFS;
			break;
		case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
			ElementsPerLine	=	2 + 8 * NUMBER_OF_JOINTS;
			break;
		case FastResearchInterface::JOINT_TORQUE_CONTROL:
			ElementsPerLine	=	2 + 3 * NUMBER_OF_JOINTS;
			break;
		default:
			return(EINVAL);
		}
	else
		ElementsPerLine = NUMBER_OF_ELEMENTS_PER_ENTRY;


	while (StopIndex >= StartIndex)
	{
		Counter++;
		fprintf(this->OutputFileHandler, "%d", Counter);
		for (i = 0; i < ElementsPerLine; i++)
		{
			fprintf(this->OutputFileHandler, "	%12.6f", this->LoggingMemory[i][StartIndex]);
		}
		fprintf(this->OutputFileHandler, "\n");
		StartIndex++;
	}

	fflush(this->OutputFileHandler);
	ReturnValue	=	fclose(this->OutputFileHandler);

	if (ReturnValue == 0)
	{
		return(EOK);
	}
	else
	{
		return(ReturnValue);
	}
}


unsigned int DataLogging2::ParseFields(unsigned int** GroupIndexes, unsigned int* GroupCounts){
	int fieldsCount	= 0;
	if (this->LoggingStrategy != NULL && stricmp((const char*)this->LoggingStrategy, "Standard")){

		InitializationFileEntry	StrategyFileParser(this->LoggingStrategy);

		int index 		= -1;
		int fieldGroup 	= -1;

		while ( StrategyFileParser.NextEntry() )
		{
			if (!stricmp (StrategyFileParser.GetValue(), "yes")){
				for (int i = 0; i < AVAILABLE_READINGS_COUNT; i++){
					if (!stricmp(StrategyFileParser.GetName(), this->Fields[3*i+1]) && !stricmp(StrategyFileParser.GetSection(), this->Fields[3*i])){
						index = i;
						break;
					}
				}

				if 			(index<24)	fieldGroup=0;
				else if 	(index<30)	fieldGroup=1;
				else if 	(index<41)	fieldGroup=2;
				else if 	(index<45)	fieldGroup=3;
				else if 	(index<49)	fieldGroup=4;
				else if 	(index<50)	fieldGroup=5;
				else if 	(index<51)	fieldGroup=6;

				fprintf(stdout,	"Field = %s, FieldGroup = %d, FieldLength = %d\n"
						, this->Fields[3*index+1], fieldGroup, this->FieldLengths[index]);

				GroupIndexes[fieldGroup][GroupCounts[fieldGroup]] = index;
				GroupCounts[fieldGroup]++;
				fieldsCount += this->FieldLengths[index];

			}
		}
		fieldsCount++;

		fprintf(stdout,	"---\nTOTAL FIELDS COUNT = %d\n---\n", fieldsCount);
	}
	else {
		strcpy(this->LoggingStrategy, "Standard");
		fieldsCount = NUMBER_OF_ELEMENTS_PER_ENTRY;
	}

	return(fieldsCount);
}

void DataLogging2::WriteFieldsDescription(FILE* OutputFile){
	int FieldIndex = 0;
	int DescriptionIndex = 0;

	fprintf(OutputFile, "counter\t");
	fprintf(OutputFile, "hosttime\t");

	for (int i=0 ; i< FIELD_DATA_TYPE_GROUPS_COUNT; i++)
		if (this->FieldGroupsCounts[i]>0)
			for (int j = 0; j < this->FieldGroupsCounts[i]; j++){

				FieldIndex 			= this->FieldGroupsIndexes[i][j];
				DescriptionIndex 	= FieldIndex*3+2;

				switch(i){
				case 0:
					fprintf(OutputFile, "%s\t", Fields[DescriptionIndex]);
					break;
				case 1: case 2: case 3: case 4:
					for (int k=0; k<this->FieldLengths[FieldIndex]; k++)
						fprintf(OutputFile, "%s[%d]\t", this->Fields[DescriptionIndex], k);
					break;
				case 5:
					for (int k=0; k<NUMBER_OF_CART_DOFS; k++)
						for (int l=0; l<NUMBER_OF_JOINTS; l++)
							fprintf(OutputFile, "%s[%d][%d]\t", this->Fields[DescriptionIndex], k, l);
					break;
				case 6:
					for (int k=0; k<NUMBER_OF_JOINTS; k++)
						for (int l=0; l<NUMBER_OF_JOINTS; l++)
							fprintf(OutputFile, "%s[%d][%d]\t", this->Fields[DescriptionIndex], k, l);
					break;
				}
			}
	fprintf(OutputFile, "\n");
}

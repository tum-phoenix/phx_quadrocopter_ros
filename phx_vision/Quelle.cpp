#include "opencv2\highgui.hpp"
#include "opencv2\aruco.hpp"
#include "opencv2\calib3d.hpp"

#include <sstream>
#include <fstream>
#include <iostream>

using namespace cv;
using namespace std;

const float calibreationSquareDimension = 0.0264f;  //in Meter, Größe der einzelnen Felder!!
const float arucoSquareDimension = 0.1321f;     //in Meter, Größe der Aruco Vierecke, NOCH NACHMESSEN!!
const Size chessboardDimension(6,9);  //Size kann zwei Werte speichern. Hier sollen es 6 Elemente in x Richtung und 9 Elemente in y Richtung sein.

void createknownBoardPositions(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) //Point3f bedeutet, dass es eine X, Y und Z Komponente gibt.
{
	for (int i=0; i<boardSize.height; i++)  //boardSize.height greift auf erste Komponente des Size Objektes zu (auf die Höhe)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f)); //zunächst wird ein Point3f Objekt erzeugt mit den Koordinaten j*squareEdgeLengths und i*squareEdgeLengths, 0.0f da z-Komponente=0, da Ebene.
		}																			  //Point3f ist eine Klasse, hier wird der Konstruktor direkt aufgerufen und die eingegebenen Werte gespeichert. 
	}																				  //kann mit Klassen so gemacht werden (ohne den Objekten einen Namen zu geben) dann können die Objekte aber nicht referenziert werden. (Hier muss das Objekt später nicht nochmal referenziert werden) 
}
// diese Funktion berechnet die Positionen der Eckpunkte der einzelnen Rechtecke in einer perfekten Wetl
// d.h keine "Verdrehung"/Kippung des Brettes auf dem die Rechtecke geklebt sind.


void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false)
{
	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		vector<Point2f> pointBuf;
		bool found = findChessboardCorners(*iter, Size(9, 6), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE );     // diese Funktion findet die Eckpunkte des chessboards. 

		if (found)
		{
			allFoundCorners.push_back(pointBuf);
		}

		if (showResults)
		{
			drawChessboardCorners(*iter, Size(9, 6), pointBuf, found);
			imshow("Looking for Corners", *iter);
			waitKey(0);
		}
	}

}
// diese Funktion soll die Eckpunkte aus den Bildern erkennen. Dafür werden die Bilder mit der Mat Variablen images eingelesen. 
// Die Koordinaten (x und y) der gefundenen Eckpunkte werden in allFoundCorners gespeichert. 
// mit der bool Variablen showResults kann ausgewählt werden, ob die Ergebnisse angezeigt werden sollen. 


void cameraCalibration(vector<Mat> &calibrationImages, Size boardSize, float squareEdgeLength, Mat &cameraMatrix, Mat &distanceCoefficients)
{
	vector<vector<Point2f>> checkerboardImageSpacePoints;  // points that are detected in the calibration image
	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false); // 1. Parameter gibt an von welchen Bildern die Punkte gefunden werden sollen, 2. Parameter gibt an wo Ergebnisse (Pos der Punkte) gespeichert werden soll, False da kein Rückgabewert erwartet wird

	vector<vector<Point3f>> worldSpaceCornerPoints(1);  // Zahl in Klammer gibt initiale Größe des Vektors an. (hier =1)
	createknownBoardPositions(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);  //create knownBoardPositions berechnet einfach die Position der Eckpunkte in einer perfekten Welt

	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);  // Verändert Größe des Vektors auf die Anzahl der Images, jedes Image dreidimensionale Points hat. Wird dann mit worldSpaceCornerPoints[0] populated. 
																									// da Ergebnisse für knownBoardPositions (perfekte Welt) für alle Images gleich sind, wird Vektor für jedes Images mit den gleichen Werten populated. 

	vector<Mat> rVectors, tVectors; 
	distanceCoefficients = Mat::zeros(8, 1, CV_64F);
	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors); //cameraMatrix und distanceCoefficients werden von der Funktioncalibrate Camera populated. 
}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients)
{
	ofstream outStream(name);
	if (outStream)
	{
		uint16_t rows = cameraMatrix.rows;
		uint16_t columns = cameraMatrix.cols; 

		outStream << rows << endl;
		outStream << columns << endl; 

		for (int r=0; r<rows; r++)
		{
			for(int c=0; c<columns; c++)
			{
				double value= cameraMatrix.at<double>(r,c);  // Wir iterieren über die einzelnen Elemente der cameraMatrix 
				outStream << value << endl; 
			}
		}

		rows = distanceCoefficients.rows;
		columns = distanceCoefficients.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r<rows; r++)
		{
			for (int c = 0; c<columns; c++)
			{
				double value = distanceCoefficients.at<double>(r, c);  // Wir iterieren über die einzelnen Elemente der cameraMatrix 
				outStream << value << endl;
			}
		}
		outStream.close();
		return true; 
	}

	return false; 
}
// diese Funktion speichert die Ergebnisse der Cameraclibration in einem File. Dabei werden die Werte von distanceCoefficients und cameraMatrix in einem File gespeichert. 

void readImages(vector<Mat> &images, int end)
{
	int i = 1;
	while (i <= end)
	{
		Mat frame;
		ostringstream image;
		image << "Kalibration" << i << ".jpg";
		frame = imread(image.str(), CV_LOAD_IMAGE_UNCHANGED);
		images.push_back(frame);
		i++;
	}
}

bool loadCameraCalibration(string name, Mat &cameraMatrix, Mat &distanceCoefficients)
{
	ifstream instream(name); 

	if (instream)
	{
		uint16_t rows;
		uint16_t columns; 

		instream >> rows; 
		instream >> columns; 

		cameraMatrix = Mat(Size(columns, rows), CV_64F);

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double read = 0.0f;
				instream >> read; 

				cameraMatrix.at<double>(r, c) = read; 
				cout << cameraMatrix.at<double>(r, c) << "\n";
			}
		}

		instream >> rows; 
		instream >> columns; 

		distanceCoefficients = Mat::zeros(rows, columns, CV_64F);

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double read = 0.0f;
				instream >> read;
				distanceCoefficients.at<double>(r, c) = read;
				cout << distanceCoefficients.at<double>(r, c) << "\n";
			}
		}
		instream.close();
		return true;

	}
	return false;


}

void CameraCalibrationProcess(Mat &cameraMatrix, Mat &distanceCoefficients, int numberOfPics)
{
	vector<Mat> imageVector;

	readImages(imageVector, numberOfPics);

	cameraCalibration(imageVector, chessboardDimension, calibreationSquareDimension, cameraMatrix, distanceCoefficients);
	saveCameraCalibration("ILoveCameraCalibration.txt", cameraMatrix, distanceCoefficients);
}

void LiveCalibration()
{
	Mat frame;
	Mat drawtoFrame;

	VideoCapture cap(0);

	if (!cap.isOpened())
	{
		return;
	}

	int framesperSecond = 20;

	namedWindow("WebCam", CV_WINDOW_AUTOSIZE);

	while (char(waitKey(1000 / framesperSecond)) != 'q' && cap.isOpened())
	{
		vector<Vec2f> foundPoints;

		cap >> frame;
		imshow("WebCam", frame);
		bool found = false;

		found = findChessboardCorners(frame, chessboardDimension, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);
		frame.copyTo(drawtoFrame);
		drawChessboardCorners(drawtoFrame, chessboardDimension, foundPoints, found);
		if (found)
			imshow("WebCam", drawtoFrame);
		else
			imshow("WebCam", frame);
	}


	return;
}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimensions)
{

	Mat frame;
	vector<int> markerIds;              //Zahlennummer des jeweiligen Markers 
	vector<vector<Point2f>> markerCorners, rejectedCandidates;
	aruco::DetectorParameters Parameters;     // Parameter, die für die Detektion benutzt werden 
	Ptr <aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);

	VideoCapture vid(0);

	if (!vid.isOpened())
	{
		return -1;
	}

	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
	vector<Vec3d> rotationVectors, translationVectors;

	while (true)
	{
		if (!vid.read(frame))
			break;

		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
		aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);

		for (int i = 0; i < markerIds.size(); i++)
		{
			aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
		}
		imshow("Webcam", frame);
		if (waitKey(30) >= 0) break;

	}
	return 1;
}

void createArucoMarkers()
{
	Mat outputMarker;
	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	for (int i = 0; i < 50; i++)
	{
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".png";
		imwrite(convert.str(), outputMarker);
	}

}
int main()
{
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distanceCoefficients;
	int numberOfPics = 38; //Anzahl der Bilder für Kamera Kalibrierung

	//CameraCalibrationProcess(cameraMatrix, distanceCoefficients, numberOfPics);	
	//saveCameraCalibration("ILoveCameraCalibration.txt", cameraMatrix, distanceCoefficients);

	if (loadCameraCalibration("ILoveCameraCalibration.txt", cameraMatrix, distanceCoefficients))
	{
		cout << "Loading Calibration values succesful" << endl;
	}

	else
	{
		cout << "loading Calibration values failed" << endl;
	}

	startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension);
	
    

	return 0;
}




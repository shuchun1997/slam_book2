CSDN上拷贝的一个demo,比较综合,但是没有调试.有时间可以研究一下,有利于理解回换检测的工作过程

#include<iostream>
#include<opencv2/opencv.hpp>
#include<DBoW3/DBoW3.h>
#include<string>
using namespace std;
using namespace cv;

int main(int argc,char** argv)
{
    try
    {
        cout<<"reading images..."<<endl;
        vector<Mat> images;
        int n=stoi(argv[1]);
        for(int i=1;i<=n;i++){
            string path="./data/"+to_string(i)+".png";
            images.push_back(imread(path));
        }

        cout<<"extracting "<<string(argv[2])<<" features!";
        Ptr<Feature2D> detector;
        string descriptor=argv[2];
        if(descriptor=="orb") 
            detector=ORB::create();
        else if(descriptor=="brisk") 
            detector=BRISK::create();
        else 
            throw runtime_error("Invide Descriptor!");

        vector<Mat> descriptors(n);
        int i=0;
        for(Mat& image:images){
            vector<KeyPoint> keypoints;
            detector->detectAndCompute(image,Mat(),keypoints,descriptors[i++]); 
        }
        
        cout<<"creating vocabulary ..."<<endl;
        const int k = 10;   
        const int L = 5;
        const DBoW3::WeightingType weight = DBoW3::TF_IDF;
        const DBoW3::ScoringType score = DBoW3::L1_NORM;
        DBoW3::Vocabulary vocab(k, L, weight, score);
        vocab.create(descriptors);
        vocab.save("vocabulary.yml.gz");    //保存训练好的词典
        cout<<"vocabulary created done!"<<endl;

        //或加载已有训练好的词典
        //DBoW3::Vocabulary voc("small_voc.yml.gz");
        //voc.load("small_voc.yml.gz");   
        cout<<"vocabulary info: "<< endl << vocab <<endl;

        cout<<"Matching iamges aginst themselves(0 low,1 high):"<<endl;
        DBoW3::BowVector v1,v2;    
        for(size_t i=0;i<descriptors.size();i++){
            vocab.transform(descriptors[i],v1);        //图像转成词袋向量
            for(size_t j=0;j<descriptors.size();j++){
                vocab.transform(descriptors[j],v2); 
                double score=vocab.score(v1,v2);     //相似性比较
                cout<<"Image "<<i<<" vs Image "<<j<<": "<<score<<endl;
            }
        }

        cout << "Creating a small database..." << endl;
        DBoW3::Database db(vocab, true, 3); // 使用正序索引，层数为3最佳
        for (size_t i = 0; i < descriptors.size(); i++)
			db.add(descriptors[i]);
		cout << "Database done!" << endl;

		cout << "Database information: " << endl << db << endl;     
        cout << "Querying the database: " << endl;      //逆序查询
		DBoW3::QueryResults ret;
		for (size_t i = 0; i < descriptors.size(); i++)
		{
			db.query(descriptors[i], ret, 4); //离目标图像最近的4张图像
			cout << "Searching for Image " << i << ". " << ret << endl;    //对ret的<<重载过
		}

        //正序的特征点匹配
        DBoW3::FeatureVector fv1, fv2;
        fv1 = db.retrieveFeatures(0);    //排过序的
        fv2 = db.retrieveFeatures(1);
        cout<<fv1.size()<<endl;
        
        DBoW3::FeatureVector::const_iterator fv1_begin = fv1.begin();             
        DBoW3::FeatureVector::const_iterator fv2_begin = fv2.begin();             
        DBoW3::FeatureVector::const_iterator fv1_end = fv1.end();             
        DBoW3::FeatureVector::const_iterator fv2_end = fv2.end();             
                                                                            
        while(fv1_begin != fv1_end && fv2_begin != fv2_end)                                         
        {                                                                            
            //分别取出属于同一node的ORB特征点(只有属于同一node，才有可能是匹配点)                       
            if(fv1_begin->first == fv2_begin->first)                                           
            {                                                                       
                const vector<unsigned int> vIndicesfv1 = fv1_begin->second;               
                const vector<unsigned int> vIndicesfv2 = fv2_begin->second;                 
                                                                            
                //遍历KF中属于该node的特征点                                                 
                for(size_t ifv1=0; ifv1<vIndicesfv1.size(); ifv1++)                     
                {                                                                   
                    const unsigned int realIdxKF = vIndicesfv1[ifv1];                                                                                             
                    
                }
            }
            else 
                fv1_begin->first < fv2_begin->first ? fv1_begin++ :fv2_begin++;
            
        }

        cout << "Saving database..." << endl;
		db.save("small_db.yml.gz");
		cout << "... done!" << endl;

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    return 0;
}


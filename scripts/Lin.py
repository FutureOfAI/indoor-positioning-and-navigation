#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      YaoLei
#
# Created:     05/05/2019
# Copyright:   (c) YaoLei 2019
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import pandas as pd
import numpy as np
import re
from sklearn.ensemble import RandomForestClassifier
from sklearn.naive_bayes import BernoulliNB

# 1.	首先我們先將資料匯入python內，我們會用到的pandas，pandas對處理這種文字資料滿好用的。
filepath='C:\\Users\\YaoLei\\Desktop\\python_test/spam.csv'
def readData_rawSMS(filepath):
	data_rawSMS = pd.read_csv(filepath,usecols=[0,1],encoding='latin-1')
	data_rawSMS.columns=['label','content']
	return data_rawSMS
data_rawSMS = readData_rawSMS(filepath)

# kaggle的'spam.csv'將我範例非垃圾郵件的label寫的genuine改成ham
# 所以如果要直接用我的程式，最簡單的方式就是ham改回genuine
for i in range(data_rawSMS.shape[0]):
    if data_rawSMS.iloc[i].label == 'ham':
        data_rawSMS.iloc[i].label='genuine'

#2.	將資料分成Train和Test
def Separate_TrainAndTest(data_rawSMS):
    n=int(data_rawSMS.shape[0])
    tmp_train=(np.random.rand(n)>=0.5)
    return data_rawSMS.iloc[np.where(tmp_train==True)[0]], data_rawSMS.iloc[np.where(tmp_train==False)[0]]
data_rawtrain,data_rawtest=Separate_TrainAndTest(data_rawSMS)

#3. 從training data去著手算哪些「詞」重要。
def generate_key_list(data_rawtrain, size_table=200,ignore=3):
    dict_spam_raw = dict()
    dict_genuine_raw = dict()
    dict_IDF = dict()

	# ignore all other than letters.
    for i in range(data_rawSMS.shape[0]):
        finds = re.findall('[A-Za-z]+', data_rawSMS.iloc[i].content)
        if data_rawSMS.iloc[i].label == 'spam':
            for find in finds:
                if len(find)<ignore: continue
                find = find.lower() #英文轉成小寫
                try:
                    dict_spam_raw[find] = dict_spam_raw[find] + 1
                except:
                    dict_spam_raw[find] = dict_spam_raw.get(find,1)
                    dict_genuine_raw[find] = dict_genuine_raw.get(find,0)
        else:
            for find in finds:
                if len(find)<ignore: continue
                find = find.lower()
                try:
                    dict_genuine_raw[find] = dict_genuine_raw[find] + 1
                except:
                    dict_genuine_raw[find] = dict_genuine_raw.get(find,1)
                    dict_spam_raw[find] = dict_spam_raw.get(find,0)

        word_set = set()
        for find in finds:
            if len(find)<ignore: continue
            find = find.lower()
            if not(find in word_set):
                try:
                    dict_IDF[find] = dict_IDF[find] + 1
                except:
                    dict_IDF[find] = dict_IDF.get(find,1)
            word_set.add(find)
    word_df = pd.DataFrame(list(zip(dict_genuine_raw.keys(),dict_genuine_raw.values(),dict_spam_raw.values(),dict_IDF.values())))
    word_df.columns = ['keyword','genuine','spam','IDF']
    word_df['genuine'] = word_df['genuine'].astype('float')/data_rawtrain[data_rawtrain['label']=='genuine'].shape[0]
    word_df['spam'] = word_df['spam'].astype('float')/data_rawtrain[data_rawtrain['label']=='spam'].shape[0]
    word_df['IDF'] = np.log10(word_df.shape[0]/word_df['IDF'].astype('float'))
    word_df['genuine_IDF'] = word_df['genuine']*word_df['IDF']
    word_df['spam_IDF'] = word_df['spam']*word_df['IDF']
    word_df['diff']=word_df['spam_IDF']-word_df['genuine_IDF']
    selected_spam_key = word_df.sort_values('diff',ascending=False)
    keyword_dict = dict()
    i = 0
    for word in selected_spam_key.head(size_table).keyword:
        keyword_dict.update({word.strip():i})
        i+=1
    return keyword_dict

# build a tabu list based on the training data
size_table = 300                 # how many features are used to classify spam
word_len_ignored = 3            # ignore those words shorter than this variable
keyword_dict=generate_key_list(data_rawtrain, size_table, word_len_ignored)

# 4.將Train資料和Test資料轉換成特徵向量
def convert_Content(content, keyword_dict):
	m = len(keyword_dict)
	res = np.int_(np.zeros(m))
	finds = re.findall('[A-Za-z]+', content)
	for find in finds:
		find=find.lower()
		try:
			i = keyword_dict[find]
			res[i]=1
		except:
			continue
	return res
def raw2feature(data_rawtrain,data_rawtest,keyword_dict):
    n_train = data_rawtrain.shape[0]
    n_test = data_rawtest.shape[0]
    m = len(keyword_dict)
    X_train = np.zeros((n_train,m));
    X_test = np.zeros((n_test,m));
    Y_train = np.int_(data_rawtrain.label=='spam')
    Y_test = np.int_(data_rawtest.label=='spam')
    for i in range(n_train):
        X_train[i,:] = convert_Content(data_rawtrain.iloc[i].content, keyword_dict)
    for i in range(n_test):
        X_test[i,:] = convert_Content(data_rawtest.iloc[i].content, keyword_dict)

    return [X_train,Y_train],[X_test,Y_test]

Train,Test=raw2feature(data_rawtrain,data_rawtest,keyword_dict)

# 5.	依據特徵資料訓練分類器
def learn(Train):
    model_NB = BernoulliNB()
    model_NB.fit(Train[0], Train[1])
    Y_hat_NB = model_NB.predict(Train[0])

    model_RF = RandomForestClassifier(n_estimators=10, max_depth=None,\
                                 min_samples_split=2, random_state=0)
    model_RF.fit(Train[0], Train[1])
    Y_hat_RF = model_RF.predict(Train[0])

    n=np.size(Train[1])
    print('Training Accuarcy NBclassifier : {:.2f}％'.format(sum(np.int_(Y_hat_NB==Train[1]))*100./n))
    print('Training Accuarcy RF: {:.2f}％'.format(sum(np.int_(Y_hat_RF==Train[1]))*100./n))
    return model_NB,model_RF
# train the Random Forest and the Naive Bayes Model using training data
model_NB,model_RF=learn(Train)

def main():
    pass

if __name__ == '__main__':
    main()

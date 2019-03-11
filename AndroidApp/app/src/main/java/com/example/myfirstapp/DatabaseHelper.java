package com.example.myfirstapp;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

public class DatabaseHelper extends SQLiteOpenHelper {

     private static final int  DATABASE_VERSION = 1;
     private static final String  DATABASE_NAME = "coaches.db";
    private static final String  TABLE_NAME = "coaches";
    private static final String  COLUMN_ID = "id";
    private static final String  COLUMN_NAME = "name";
    private static final String  COLUMN_EMAIL = "email";
    private static final String  COLUMN_PASS = "pass";
    SQLiteDatabase db;
    private static final  String TABLE_CREATE="create table coaches (id integer primary key not null auto_increment , " + " name text not null,email text not null, pass text not null);";

    public DatabaseHelper(Context context){

       super(context, DATABASE_NAME, null,  DATABASE_VERSION);


    }

    @Override
    public void onCreate(SQLiteDatabase db) {
          db.execSQL(TABLE_CREATE);
          this.db=db;
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        String query="DROP TABLE IF EXISTS " + TABLE_NAME;
       db.execSQL(query);
       this.onCreate(db);
    }

    public void insertContact(Coach c ){
       db=this.getWritableDatabase();
        ContentValues val=new ContentValues();
        val.put(COLUMN_NAME, c.getName());
        val.put(COLUMN_EMAIL, c.getEmail());
        val.put(COLUMN_PASS, c.getPassword());
        db.insert(TABLE_NAME, null, val);

    }


    public String searchpass(String email){
        db=this.getReadableDatabase();
        String query="select pass from " + TABLE_NAME;
        Cursor curs= db.rawQuery(query,null);
        String retpass="not_matched";
        String a;
        if (curs.moveToFirst()){
            do {

            a=curs.getString(2);
            if(a==email){
                retpass=curs.getString(3);
                break;

            }


            }while(curs.moveToNext());


        }
        return retpass;

    }

}

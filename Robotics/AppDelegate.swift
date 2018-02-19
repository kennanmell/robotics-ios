//
//  AppDelegate.swift
//  Robotics
//
//  Created by Kennan Mell on 2/14/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate {

    var window: UIWindow?
    weak static var mvc: MainViewController?


    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplicationLaunchOptionsKey: Any]?) -> Bool {
        //Settings.instance.serverIp = "localhost"
        //Settings.instance.serverPort = 9995
        return true
    }

    func applicationWillResignActive(_ application: UIApplication) {
        Settings.save()
    }

    func applicationDidEnterBackground(_ application: UIApplication) {
        AppDelegate.mvc?.didEnterBackground()
        RequestHandler.instance.paired = false
        RequestHandler.instance.send(command: Commands.kill)
    }

    func applicationWillEnterForeground(_ application: UIApplication) {
        RequestHandler.instance.connectToServer()
        AppDelegate.mvc?.willEnterForeground()
    }

    func applicationDidBecomeActive(_ application: UIApplication) {
        // Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
    }

    func applicationWillTerminate(_ application: UIApplication) {
        // Called when the application is about to terminate. Save data if appropriate. See also applicationDidEnterBackground:.
    }


}


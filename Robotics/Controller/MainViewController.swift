//
//  MainViewController.swift
//  Robotics
//
//  Created by Kennan Mell on 2/14/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class MainViewController: UIViewController, UITableViewDelegate, UITableViewDataSource {
    
    var mainView: MainView {
        return self.view as! MainView
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()

        mainView.settingsButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(MainViewController.settingsPressed)))
        
        mainView.speakButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(MainViewController.speakPressed)))

        mainView.pairButton.addGestureRecognizer(
            UITapGestureRecognizer(target: self,
                                   action: #selector(MainViewController.pairPressed)))
        
        mainView.roomTableView.delegate = self
        mainView.roomTableView.dataSource = self
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        mainView.roomTableView.reloadData()
    }
    
    @objc func settingsPressed() {
        performSegue(withIdentifier: "MainToSettings", sender: self)
    }
    
    @objc func speakPressed() {
        // TODO: send request
        performSegue(withIdentifier: "MainToStatus", sender: self)
    }
    
    @objc func pairPressed() {
        // TODO: send request
        performSegue(withIdentifier: "MainToStatus", sender: self)
    }
    
    // MARK: UITableViewDelegate
    
    // MARK: UITableViewDataSource
    
    func numberOfSections(in tableView: UITableView) -> Int {
        return 1
    }
    
    func tableView(_ tableView: UITableView,
                   titleForHeaderInSection section: Int) -> String? {
        return "Go To Room"
    }
    
    func tableView(_ tableView: UITableView,
                   numberOfRowsInSection section: Int) -> Int {
        return Settings.instance.roomArray.count
    }
    
    func tableView(_ tableView: UITableView,
                   cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let result = UITableViewCell()
        result.textLabel?.text = Settings.instance.roomArray[indexPath.row]
        result.accessoryType = .disclosureIndicator
        return result
    }
}

